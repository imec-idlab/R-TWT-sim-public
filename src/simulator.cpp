#include <iostream>
#include <fstream>
#include <random>
#include <map>
#include <functional>
#include <queue>
#include <memory>

#include "../include/json.hpp"

std::multimap<double, std::function<void ()>> g_eventQueue;
typedef std::multimap<double, std::function<void ()>>::iterator Event;

int NDBPS[2][12] = {{117, 234, 351, 468, 702, 936, 1053, 1170, 1404, 1560, 1755, 1950},
                    {234, 468, 702, 936, 1404, 1872, 2106, 2340, 2808, 3120, 3510, 3900}};

bool g_saveDelaySamples = false;

double g_curTime = 0;
double g_simTime = 1.e8; //us
double g_errorProb = 0.; //probability of failure due to channel errors

int g_edcaNumRetx = 32; //including initial TX

int g_twtNumRetx = 0; //including initial TX
int g_numOfPktsInTwt = 1; //number of packet to be served within TWT SP

Event Schedule (double delay, std::function<void ()> handler)
{
  return g_eventQueue.insert (std::pair<double, std::function<void ()>>(g_curTime + delay, handler));
}

void Cancel (Event& ev)
{
  if (ev != g_eventQueue.end ())
    g_eventQueue.erase (ev);
  ev = g_eventQueue.end ();
}

double CalculatePacketDuration (int bandwidth/*MHz*/, int packetSize /*bytes*/, int mcsIndex) //in us
{
  double res = 0;
  res += 20 /*legacy*/ + 4 /*RL-SIG*/ + 8 /*SIG-A*/+ 4 /*HE-STF*/ + 4/*HE-LTF*/;
  int N_dbps = NDBPS[bandwidth / 20 - 1][mcsIndex];
  res += ceil ((double)(16 + packetSize * 8 + 6) / (double)N_dbps) * 13.2;
  return res;
}

int g_seed = 1;
std::default_random_engine g_generator;

double g_slot = 9.; //us
double g_sifs = 16.; //us
double g_eifs = 64.; //us
double g_ack = 32.; //us
double g_ackTimeout = g_ack + g_sifs; //us, default (can be rewritten in config)
double g_twtGuard = 0.; //us

double g_twtPktSize = 200.; //bytes
double g_edcaPktSize = 1500.; //bytes

double g_twtSpPeriod = 16000.; //us

int g_mcsIndex = 5;
int g_bandwidth = 40; //MHz

struct TWT
{
  double startTime;
  double endTime;
};

std::queue<TWT> g_closestTwts;

//---------------------------------------------------------------------------------------

class TWT_STA: public std::enable_shared_from_this<TWT_STA>
{
public:
  double m_spOffset; //time of first SP, us
  double m_spDuration; //us
  double m_spPeriod; //us

  double m_firstPacketTime; //us
  double m_txDuration; //us
  double m_packetInterval; //us

  bool m_randomGeneration; //true = Poisson, false = CBR

  int m_maxQueueSize; //in pkts
  int m_queueSize; //in pkts

  std::queue<double> m_pktTimestamps; //in us
  std::vector<double> m_pktDelays; //in us

  std::vector<double> m_queueSizePdf; //queue size at the end of service/vacation PDF
  
  std::vector<double> m_queueSizeTimeIntervals; //time duration for each queue size
  double m_lastQueueSizeUpdate; //save curTime for each update

  bool m_inTransmitPhase;
  double m_twtSpEndTime;

  int m_curAttempt;
  int m_numReTx; //including initial

  int m_successPkts;
  int m_failedPkts;
  int m_droppedPkts;

  TWT_STA () : m_maxQueueSize (100), m_queueSize (0), m_lastQueueSizeUpdate (0), m_inTransmitPhase (false), m_twtSpEndTime (0.),
    m_curAttempt (0), m_numReTx (0), m_successPkts (0), m_failedPkts (0), m_droppedPkts (0)
  {
    m_queueSizePdf.resize (m_maxQueueSize+1, 0);
    m_queueSizeTimeIntervals.resize (m_maxQueueSize+1, 0);
  }

  void NewPacket ();
  void StartTwt ();
  void EndTwt ();
  void Start ();
  void QueueSizeUpdate ();
  void TransmitPhase ();
  void FinishTransmit (bool success);
  void ReportQueueSize ();
};

//---------------------------------------------------------------------------------------

class EDCA_STA : public std::enable_shared_from_this<EDCA_STA>
{
public:
  int m_id;

  int m_cwMin;
  int m_cwMax;
  int m_numReTx; //including initial
  int m_AIFSN;

  int m_backoff;
  int m_cw;
  int m_curAttempt;

  double m_txDuration;
  double m_packetInterval;

  int m_queueSize; //in pkts
  std::queue<double> m_pktTimestamps; //in us

  Event m_countdownEvent;

  int m_successPkts;
  int m_failedPkts;
  double m_sumPktDelays;
  double m_curPktQueuingDelay;
  double m_sumPktServiceDelays;

  bool m_randomGeneration;

  EDCA_STA () : m_id (0), m_cwMin (16), m_cwMax (1024), m_numReTx (7), m_AIFSN (2), m_cw (16), m_curAttempt (0), m_queueSize (0),
    m_countdownEvent (g_eventQueue.end ()), m_successPkts (0), m_failedPkts (0), m_sumPktDelays (0.), 
    m_curPktQueuingDelay (0.), m_sumPktServiceDelays (0.), m_randomGeneration (true) {};

  void DoubleCw ();
  void ResetCw ();
  void StartBackoff ();
  void Start ();
  void NewPacket ();
  void BackoffCountdown ();
  void FreezeBackoff ();
  void RestoreBackoff (double delay);
  void TransmitPacket ();
};

std::vector<std::shared_ptr<EDCA_STA>> g_allEdcaStas;
std::vector<std::shared_ptr<TWT_STA>> g_allTwtStas;

bool IsCoinSuccess ()
{
  std::uniform_real_distribution<> var(0., 1.);
  double coin = var(g_generator);
  return (coin > g_errorProb);
}

//---------------------------------------------------------------------------------------

class Channel
{
public:
  Channel (): m_finishEvent (g_eventQueue.end ()) {};

  std::vector<std::shared_ptr<EDCA_STA>> m_txStas;

  Event m_finishEvent;

  void AddSta (std::shared_ptr<EDCA_STA> sta)
    {
      m_txStas.push_back (sta);
      if (m_finishEvent == g_eventQueue.end ())
        {
          std::function <void ()> callback = std::bind (&Channel::FinishTx, this);
          m_finishEvent = Schedule (sta->m_txDuration, callback);
          for (std::shared_ptr<EDCA_STA>& station : g_allEdcaStas)
            {
              std::function <void ()> callback1 = std::bind (&EDCA_STA::FreezeBackoff, station.get());
              Schedule (1.e-3, callback1);
            }
        }
    }

  void FinishTx ()
    {
      double closestStart = g_simTime;
      if (!g_closestTwts.empty ())
        {
          TWT closest = g_closestTwts.front ();
          closestStart = closest.startTime;
        }
      if (m_txStas.size () == 1)
        {
          for (std::shared_ptr<EDCA_STA>& station : m_txStas)
            {
              station->m_queueSize--;
              station->m_successPkts++;

              station->m_sumPktDelays += g_curTime - station->m_pktTimestamps.front ();
              station->m_sumPktServiceDelays += (g_curTime - station->m_pktTimestamps.front ()) - station->m_curPktQueuingDelay;
              
              station->m_pktTimestamps.pop ();

              station->m_curAttempt = 0;
              station->ResetCw ();
            }
          for (std::shared_ptr<EDCA_STA>& station : g_allEdcaStas)
            {
              if (station->m_queueSize && g_curTime + g_sifs + g_ack + g_sifs + station->m_AIFSN * g_slot < closestStart)
                {
                  station->RestoreBackoff (g_sifs + g_ack + g_sifs + station->m_AIFSN * g_slot);
                }
            }  
        }
      else
        {
          for (std::shared_ptr<EDCA_STA>& station : m_txStas)
            {
              if (station->m_curAttempt < station->m_numReTx || !station->m_numReTx)
                {
                  station->DoubleCw ();
                }
              else
                {
                  station->ResetCw ();
                  station->m_curAttempt = 0;
                  station->m_queueSize--;
                  station->m_failedPkts++;

                  station->m_pktTimestamps.pop ();
                }
            }  
          for (std::shared_ptr<EDCA_STA>& station : g_allEdcaStas)
            {
              if (station->m_queueSize && g_curTime + g_eifs + station->m_AIFSN * g_slot < closestStart)
                {
#ifdef DEBUG
                  std::cout << g_curTime << "us: call restore backoff with delay = " << g_eifs + station->m_AIFSN * g_slot << std::endl;
#endif
                  station->RestoreBackoff (g_eifs + station->m_AIFSN * g_slot);
                }
            }
        }
      m_txStas.clear ();
      m_finishEvent = g_eventQueue.end ();
    }
};

Channel g_channel;

//---------------------------------------------------------------------------------------

void EDCA_STA::DoubleCw ()
  {
    m_cw = std::min(m_cwMax, m_cw * 2);
  }

void EDCA_STA::ResetCw ()
  {
    m_cw = m_cwMin;
  }

void EDCA_STA::StartBackoff ()
  {
    std::uniform_int_distribution<> var (0, m_cw - 1);
    m_backoff = var (g_generator);
#ifdef DEBUG
    std::cout << g_curTime << "us: generate backoff = " << m_backoff << " from window = " << m_cw << "for STA# " << m_id << std::endl;
#endif

    if (m_queueSize > 0 && !m_curAttempt)
      {
        m_curPktQueuingDelay = g_curTime - m_pktTimestamps.front ();
      }

    if (!m_backoff)
      {
        m_countdownEvent = g_eventQueue.end ();
        TransmitPacket ();
      }
    else
      {
        std::function <void ()> callback = std::bind (&EDCA_STA::BackoffCountdown, this);
        m_countdownEvent = Schedule (g_slot, callback);
      }
  }

void EDCA_STA::Start ()
  {
    //std::uniform_real_distribution<> var (0, m_packetInterval);
    std::function <void ()> callback = std::bind (&EDCA_STA::NewPacket, this);
    Schedule (g_slot, callback);
    //Schedule (var (g_generator), callback);
  }

void EDCA_STA::NewPacket ()
  {
    m_queueSize++;
    m_pktTimestamps.push (g_curTime);
    if (m_queueSize == 1)
      {
        StartBackoff ();
      }
    std::function <void ()> callback = std::bind (&EDCA_STA::NewPacket, this);
    double delay = m_packetInterval;
    if (m_randomGeneration)
      {
        std::exponential_distribution<> var (1. / m_packetInterval);
        delay = var(g_generator);
      }
    Schedule (delay, callback);
  }

void EDCA_STA::BackoffCountdown ()
  {
    if (!m_backoff && m_queueSize)
      {
        StartBackoff ();
        return;
      }
    if (!m_queueSize)
      {
        m_countdownEvent = g_eventQueue.end ();
        return;
      }
    m_backoff--;
#ifdef DEBUG
    std::cout << g_curTime << "us: backoff countdown STA #" << m_id << ", cur val = " << m_backoff << std::endl;
#endif
    if (!m_backoff)
      {
        m_countdownEvent = g_eventQueue.end ();
        TransmitPacket ();
      }
    else
      {
        std::function <void ()> callback = std::bind (&EDCA_STA::BackoffCountdown, this);
        m_countdownEvent = Schedule (g_slot, callback);
      }
  }

void EDCA_STA::FreezeBackoff ()
  {
#ifdef DEBUG
    std::cout << g_curTime << "us: freeze backoff for STA# " << m_id << std::endl;
#endif
    Cancel (m_countdownEvent);
  }

void EDCA_STA::RestoreBackoff (double delay)
  {
#ifdef DEBUG
    std::cout << g_curTime + delay << "us: restore backoff" << std::endl;
#endif
    if (!m_queueSize) return;
    std::function <void ()> callback = std::bind (&EDCA_STA::BackoffCountdown, this);
    m_countdownEvent = Schedule (delay, callback);
  }

void EDCA_STA::TransmitPacket ()
  {
#ifdef DEBUG
    std::cout << g_curTime << "us: TX pkt from STA# " << m_id << " tx dur = " << m_txDuration << "us" << std::endl;
#endif
    m_curAttempt++;
    if (!g_closestTwts.empty ())
      {
        TWT closest = g_closestTwts.front ();
        if (g_curTime + m_txDuration + g_sifs + g_ack <= closest.startTime)
          {
            g_channel.AddSta (shared_from_this ());
          } 
      }
    else
      {
        g_channel.AddSta (shared_from_this ());
      }
  }

//---------------------------------------------------------------------------------------  

void TWT_STA::NewPacket ()
  {
    if (m_queueSize < m_maxQueueSize)
      {
        QueueSizeUpdate ();
        m_queueSize++;
        m_pktTimestamps.push (g_curTime);
      }
    else
      {
        m_droppedPkts++;
      }

    if (m_queueSize == 1 && m_inTransmitPhase)
      {
        TransmitPhase ();
      }

    std::function <void ()> callback = std::bind (&TWT_STA::NewPacket, this);
    double delay = m_packetInterval;
    if (m_randomGeneration)
      {
        std::exponential_distribution<> var (1. / m_packetInterval);
        delay = var(g_generator);
      }
    Schedule (delay, callback);
  }

void TWT_STA::StartTwt ()
  {
    m_twtSpEndTime = g_curTime + m_spDuration;

    std::function <void ()> callback = std::bind (&TWT_STA::EndTwt, this);
    Schedule (m_spDuration, callback);

    m_queueSizePdf[m_queueSize]++;

    for (auto& station : g_allEdcaStas)
      {
        station->FreezeBackoff ();
      }
    
    m_inTransmitPhase = true;

    std::function <void ()> callback2 = std::bind (&TWT_STA::ReportQueueSize, this);
    Schedule (m_txDuration + g_sifs + g_ack, callback2);

    TransmitPhase ();
  }

void TWT_STA::EndTwt ()
  {
    m_inTransmitPhase = false;
    g_closestTwts.pop ();
    TWT sample = {g_curTime + m_spPeriod - m_spDuration, g_curTime + m_spPeriod};
    g_closestTwts.push (sample);

    TWT closest = g_closestTwts.front ();
    for (auto& station : g_allEdcaStas)
      {
        if (station->m_queueSize && g_curTime + station->m_txDuration + g_sifs + g_ack <= closest.startTime)
          {
            station->StartBackoff ();
          }
      }

    std::function <void ()> callback = std::bind (&TWT_STA::StartTwt, this);
    Schedule (m_spPeriod - m_spDuration, callback);
  }

void TWT_STA::Start ()
  {
    TWT sample = {m_spOffset, m_spOffset + m_spDuration};
    g_closestTwts.push (sample);
    std::function <void ()> callback = std::bind (&TWT_STA::NewPacket, this);
    Schedule (m_firstPacketTime, callback);

    std::function <void ()> callback1 = std::bind (&TWT_STA::StartTwt, this);
    Schedule (m_spOffset, callback1);
  }

void TWT_STA::QueueSizeUpdate ()
  {
    m_queueSizeTimeIntervals.at (m_queueSize) += g_curTime - m_lastQueueSizeUpdate;
    m_lastQueueSizeUpdate = g_curTime;
  }

void TWT_STA::TransmitPhase ()
  {
    if (g_curTime + m_txDuration + g_sifs + g_ack <= m_twtSpEndTime && m_queueSize)
      {
        m_curAttempt++;
        bool success = IsCoinSuccess ();
        std::function <void ()> callback = std::bind (&TWT_STA::FinishTransmit, this, success);
        if (success)
          {
            Schedule (m_txDuration + g_sifs + g_ack - 1.e-3, callback);
          }
        else
          {
            Schedule (m_txDuration + g_ackTimeout - 1.e-3, callback);
          }
      }
  }

void TWT_STA::ReportQueueSize ()
  {
    m_queueSizePdf[m_queueSize]++;
    if (g_curTime + m_txDuration + g_sifs + g_ack <= m_twtSpEndTime)
      {
        std::function <void ()> callback = std::bind (&TWT_STA::ReportQueueSize, this);
        Schedule (m_txDuration + g_sifs + g_ack, callback);
      }
  }

void TWT_STA::FinishTransmit (bool success)
  {
    if (m_queueSize)
      {
        if (success)
          {
            QueueSizeUpdate ();
            m_queueSize--;
            m_successPkts++;
            m_curAttempt = 0;
            m_pktDelays.push_back (g_curTime - m_pktTimestamps.front ());
            m_pktTimestamps.pop ();
          }
        else //failure
          {
            if (m_curAttempt >= m_numReTx && m_numReTx) //last attempt
              {
                m_curAttempt = 0;
                QueueSizeUpdate ();
                m_queueSize--;
                m_failedPkts++;
                m_pktTimestamps.pop ();
              }
          }
      }
    
    if (m_queueSize)
      {
        TransmitPhase ();
      }
  }

//---------------------------------------------------------------------------------------

double calcMean (std::vector<double>& v)
{
  double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
  return sum / v.size();
}

double calcStdDev (std::vector<double>& v, double m)
{
  double sum = 0.0;
  std::for_each (std::begin(v), std::end(v), [&](const double s) {
    sum += (s - m) * (s - m);
  });

  return sqrt(sum / (v.size()-1));
}

int main()
{
  int numEdca = 1, numTwt = 0;
  double edcaPktInterval = 10, twtPktInterval = 16000; //us

  //----------Reading params from config -----------------------
  std::ifstream config ("config.json");
  std::stringstream buffer;
  buffer << config.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());
  
  auto saveDelaysIt = json.find ("saveDelays");
  if (saveDelaysIt != json.end ())
    {
      g_saveDelaySamples = *saveDelaysIt;
    }

  auto simTimeIt = json.find ("simTime");
  if (simTimeIt != json.end ())
    {
      g_simTime = *simTimeIt;
    }

  auto seedIt = json.find ("seed");
  if (seedIt != json.end ())
    {
      g_seed = *seedIt;
    }

  auto errorProbIt = json.find ("errorProb");
  if (errorProbIt != json.end ())
    {
      g_errorProb = *errorProbIt;
    }

  auto ackTimeoutIt = json.find ("ackTimeout");
  if (ackTimeoutIt != json.end ())
    {
      g_ackTimeout = *ackTimeoutIt;
      if (g_ackTimeout < g_sifs + g_ack)
        {
          std::cerr << "WRONG PARAM: ACK TIMEOUT < SIFS + ACK" << std::endl;
          exit (-1);
        }
    }

  //EDCA STA params
  for (auto edcaOption : json["edca"].items ())
    {
      if (edcaOption.key () == "numSta")
        {
          numEdca = edcaOption.value ().get<int> ();
        }
      if (edcaOption.key () == "interval")
        {
          edcaPktInterval = edcaOption.value ().get<double> ();
        }
      if (edcaOption.key () == "pktSize")
        {
          g_edcaPktSize = edcaOption.value ().get<int> ();
        }
      if (edcaOption.key () == "numRetx")
        {
          g_edcaNumRetx = edcaOption.value ().get<int> ();
        }
    }

  //TWT STA params
  for (auto twtOption : json["twt"].items ())
    {
      if (twtOption.key () == "numSta")
        {
          numTwt = twtOption.value ().get<int> ();
        }
      if (twtOption.key () == "interval")
        {
          twtPktInterval = twtOption.value ().get<double> ();
        }
      if (twtOption.key () == "pktSize")
        {
          g_twtPktSize = twtOption.value ().get<int> ();
        }
      if (twtOption.key () == "spPeriod")
        {
          g_twtSpPeriod = twtOption.value ().get <double> ();
        }
      if (twtOption.key () == "spDurationPkts")
        {
          g_numOfPktsInTwt = twtOption.value ().get <int> ();
        }
      if (twtOption.key () == "numRetx")
        {
          g_twtNumRetx = twtOption.value ().get<int> ();
        }
    }
  
  //-----------------------------------------------------------
  g_generator.seed (g_seed);

  for (int i = 0; i < numEdca; i++)
    {
      std::shared_ptr<EDCA_STA> sta (new EDCA_STA ());
      sta->m_txDuration = CalculatePacketDuration (g_bandwidth, g_edcaPktSize, g_mcsIndex); //us
      sta->m_packetInterval = edcaPktInterval; //us
      sta->m_id = i;
      sta->m_numReTx = g_edcaNumRetx;
      g_allEdcaStas.push_back (sta);
      sta->Start ();
    }

  for (int i = 0; i < numTwt; i++)
    {
      std::shared_ptr<TWT_STA> sta (new TWT_STA ());
      sta->m_txDuration = CalculatePacketDuration (g_bandwidth, g_twtPktSize, g_mcsIndex); // us
      sta->m_spDuration = (sta->m_txDuration + std::max(g_sifs + g_ack, g_ackTimeout)) * g_numOfPktsInTwt + g_twtGuard;
      sta->m_firstPacketTime = i * sta->m_spDuration;
      sta->m_packetInterval = twtPktInterval; // us
      sta->m_spOffset = sta->m_firstPacketTime;
      sta->m_spPeriod = g_twtSpPeriod;
      sta->m_randomGeneration = true; //for Poisson
      sta->m_numReTx = g_twtNumRetx;
      g_allTwtStas.push_back (sta);
      sta->Start ();
    }

  while (g_curTime <= g_simTime && !g_eventQueue.empty ())
    {
      auto it = g_eventQueue.begin ();
      if (it->first > g_simTime)
        {
          g_eventQueue.clear ();
          break;
        }
      g_curTime = it->first;
      it->second ();
      g_eventQueue.erase (g_eventQueue.begin ());
    }

  //Calculate stats

  //EDCA STAs
  if (g_allEdcaStas.size ())
    {
      double goodput = 0.;
      double totalDelay = 0.;
      double serviceDelay = 0;
      int successPkts = 0;
      int failedPkts = 0;
      for (auto& station : g_allEdcaStas)
        {
          goodput += station->m_successPkts * g_edcaPktSize;
          totalDelay += station->m_sumPktDelays;
          serviceDelay += station->m_sumPktServiceDelays;
          successPkts += station->m_successPkts;
          failedPkts += station->m_failedPkts;
        }
      goodput /= g_simTime;
      totalDelay /= successPkts;
      serviceDelay /= successPkts;

#ifdef DEBUG
      std::cout << 8.*goodput << " " << totalDelay / 1000. << " " << serviceDelay / 1000. << " "
        << (double)failedPkts / (double)(failedPkts + successPkts) << std::endl; 
#endif
    }

  if (g_allTwtStas.size ())
    {
      for (auto& station : g_allTwtStas)
        {
          //Calculate queue size at embedded time moments PDF
          double total = 0;
          for (auto& size : station->m_queueSizePdf)
            {
              total += size;
            }
          
          for (int i = 0; i < station->m_queueSizePdf.size (); i++)
            {
              station->m_queueSizePdf.at (i) /= total;
            }

          //Calculate queue size at arbitrary time moment PDF
          station->QueueSizeUpdate ();

          std::ofstream f_queue_size_pdf;
          std::ostringstream fileNameStream;
          fileNameStream << "sim_queue_size_pdf_" << g_seed << "_"
            << (int) station->m_spPeriod << "_" << g_numOfPktsInTwt;
          if (g_twtNumRetx)
            {
              fileNameStream << "_" << g_twtNumRetx;
            }
          fileNameStream << ".dat";
          f_queue_size_pdf.open (fileNameStream.str ());
          for (int i = 0; i < station->m_queueSizeTimeIntervals.size (); i++)
            {
              station->m_queueSizeTimeIntervals.at (i) /= g_simTime;
              f_queue_size_pdf << station->m_queueSizePdf.at (i) << " " << station->m_queueSizeTimeIntervals.at (i) << std::endl;
            }
          f_queue_size_pdf.close ();

          if (g_saveDelaySamples)
            {
              //export delay samples
              std::ofstream f;
              fileNameStream.str ("");
              fileNameStream << "sim_delay_samples_" << g_seed << "_"
                << (int) station->m_spPeriod << "_" << g_numOfPktsInTwt;
              if (g_twtNumRetx)
                {
                  fileNameStream << "_" << g_twtNumRetx;
                }
              fileNameStream << ".dat";
              f.open (fileNameStream.str ());
              for (auto d : station->m_pktDelays)
                {
                  f << d << std::endl;
                }
              f.close();
            }

          //Calculate average and variance of pkt delay
          double meanDelay = calcMean (station->m_pktDelays);
          double stdDevDelay = calcStdDev (station->m_pktDelays, meanDelay);

          //Check Little's law
          double avgQueueSize = 0;
          for (int i = 0; i < station->m_queueSizeTimeIntervals.size (); i++)
            {
              avgQueueSize += i * station->m_queueSizeTimeIntervals.at (i);
            }

          double dropProb = (double)station->m_droppedPkts / (double)(station->m_successPkts + station->m_droppedPkts + station->m_failedPkts);
          double failProb = (double)station->m_failedPkts / (double)(station->m_successPkts + station->m_failedPkts);
          double adjustedLambda = (1. / station->m_packetInterval) * (1. - dropProb);
          double LittlesLawMeanDelay = avgQueueSize / adjustedLambda;

          std::ofstream f_delay;
          fileNameStream.str ("");
          fileNameStream << "sim_mean_delay_" << g_seed << ".dat";
          f_delay.open (fileNameStream.str ());
          f_delay << station->m_spPeriod << " " << g_numOfPktsInTwt << " " << g_errorProb << " " << LittlesLawMeanDelay
            << " " << meanDelay << " " << stdDevDelay << " " << failProb << " " << g_twtNumRetx << std::endl;
          f_delay.close ();
        }
    }
  
  return 0;
}
