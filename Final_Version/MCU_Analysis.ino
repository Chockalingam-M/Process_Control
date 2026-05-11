
    #include <Arduino.h>
    #include <math.h>


    #define NUM_TAPS       64
    #define SAMPLE_RATE    10000.0f
    #define PI_VAL         3.14159265359f
    #define MU_SHIFT       11
    #define Q16_SHIFT      16
    #define Q16_SCALE      (1L << Q16_SHIFT)
    #define TOTAL_SAMPLES  10000


    typedef int32_t q16_t;

    q16_t weights[NUM_TAPS];
    q16_t x_buffer[NUM_TAPS];
    q16_t y_out;
    q16_t e_out;

    //Function
    q16_t float_to_q16(float x)
    {
        return (q16_t)(x * Q16_SCALE);
    }
    float q16_to_float(q16_t x)
    {
        return ((float)x / (float)Q16_SCALE);
    }

    // LMS FILTER

    void lms_filter(q16_t x_in, q16_t d_in)
    {
        int i;
        for(i = NUM_TAPS - 1; i > 0; i--)
        {
            x_buffer[i] = x_buffer[i - 1];
        }
        x_buffer[0] = x_in;

        // FIR OUTPUT

        int64_t acc = 0;
        for(i = 0; i < NUM_TAPS; i++)
        {
            acc += ((int64_t)weights[i] * x_buffer[i]);
        }
        y_out = (q16_t)(acc >> Q16_SHIFT);

        // ERROR
        e_out = d_in - y_out;

        // LMS WEIGHT UPDATE
        for(i = 0; i < NUM_TAPS; i++)
        {
            int64_t update;
            update = (int64_t)e_out * x_buffer[i];
            update = update >> MU_SHIFT;
            update = update >> Q16_SHIFT;
            weights[i] += (q16_t)update;
        }
    }


    void setup()
    {
        Serial.begin(9600);
        for(int i = 0; i < NUM_TAPS; i++)
        {
            weights[i] = 0;
            x_buffer[i] = 0;
        }
        delay(2000);
        Serial.println("sample,time,x_in,d_in,y_out,e_out");
    }


    void loop()
    {
        static int sample = 0;


        if(sample >= TOTAL_SAMPLES)
        {
            while(1);
        }
        float t , clean , noise , noisy_input;
        t = (float)sample / SAMPLE_RATE;
        clean = sinf(2.0f * PI_VAL * SIGNAL_FREQ * t);
        noise = 0.5f * sinf(2.0f * PI_VAL * NOISE_FREQ * t);
        noisy_input = clean + noise;
        q16_t x_in_q16;
        q16_t d_in_q16;

        x_in_q16 = float_to_q16(noisy_input);
        d_in_q16 = float_to_q16(clean);

        // LMS FILTER
        lms_filter(x_in_q16, d_in_q16);


        Serial.print(sample);
        Serial.print(",");

        Serial.print(t, 6);
        Serial.print(",");

        Serial.print(q16_to_float(x_in_q16), 6);
        Serial.print(",");

        Serial.print(q16_to_float(d_in_q16), 6);
        Serial.print(",");

        Serial.print(q16_to_float(y_out), 6);
        Serial.print(",");

        Serial.println(q16_to_float(e_out), 6);

        // NEXT SAMPLE
        sample++;
        delayMicroseconds(100);  //bascially given as a 10ns to represent the clock of 20MHz
    }