#ifndef ENCODER_H
#define ENCODER_H

#define MIN_DT 0.0005f  // Increased from 0.005 to 50ms minimum

#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

class Encoder{
  public:
    Encoder(uint8_t pinA, uint8_t pinB);
    static Encoder* instances[12];

    template<int N>
    static void count() {   
      if (instances[N] != nullptr) {
          instances[N]->counting();
      }
    }

    template<int N>
    static void countDouble() {
      if (instances[N] != nullptr) {
          instances[N]->countingDouble();
      }
    }

    template<int N>
    static void countQuadrature() {
      if (instances[N] != nullptr) {
          instances[N]->countingQuadrature();
      }
    }
    
    void begin(int index);
    long getCount();
    long getCountDouble();
    long getCountQuadrature();
    void updateSpeed();
    float getCurrentSpeed();  
    float getCurrentRPS();
    void setResolution(uint32_t resolution) { encoder_resolution = resolution; }
    void setGearRatio(float ratio) { gear_ratio = ratio; }
   
   private:
    enum CountMode {
      COUNT_SINGLE,
      COUNT_DOUBLE,
      COUNT_QUADRATURE
    };

    long getActiveCount();
    IRAM_ATTR void counting();
    IRAM_ATTR void countingDouble();
    IRAM_ATTR void countingQuadrature();
    uint8_t pina;
    uint8_t pinb;
    volatile long counter;
    volatile long counterDouble;
    volatile long counterQuadrature;
    volatile int lastEncoded;

    uint32_t encoder_resolution = 540; 
    float gear_ratio = 1.0;
    float current_speed = 0;
    float current_rps = 0;
    uint32_t last_time = 0;
    float last_position = 0;
    CountMode count_mode = COUNT_SINGLE;
    bool initialized = false; 
};

#endif