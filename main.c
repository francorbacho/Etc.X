/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F27K40
        Driver Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#include <time.h>
#include <math.h>
#include <xc.h>
#include "mcc_generated_files/mcc.h"

// TODO: Check if we can measure time with "time.h" like in the example featured
// in https://en.cppreference.com/w/c/chrono/clock_t
// If we can't, try to read the oscillator and convert it to time.
// If we can't, use interrupts.

#define RANGE_START          0
#define RANGE_END            100
#define MAX_APPS_DEVIATION   0.10f

static int variable = 12;

inline int map(int value, int rangeStart, int rangeEnd, int mappedRangeStart, int mappedRangeEnd) {
    int previousRange = rangeEnd - rangeStart;
    int futureRange = mappedRangeEnd - mappedRangeStart;
    return (value - rangeStart) / previousRange * futureRange + mappedRangeStart;
}

typedef struct {
    unsigned short pinValue;
    int minValue, maxValue;
    int value;
    adcc_channel_t channel;
} sensor_data;

typedef struct {
    sensor_data sensors[2];
    // TODO: Check if this clock() function is too expensive.
    clock_t last_plausability;
} sensor_set;

// FIXME: Define things on their own.
#define APPS1 0
#define TPS1 0
#define TPS2 0

float calculate_deviation(sensor_set *);

void sensor_update(sensor_data *sensor) {
    unsigned short pinValue = ADCC_GetSingleConversion(sensor->channel);
    sensor->pinValue = pinValue;
    sensor->value = map((int) pinValue, sensor->minValue, sensor->maxValue, RANGE_START, RANGE_END);
}

bool sensor_set_over_max_implausability_time(sensor_set *set) {
    // CLOCKS / CLOCKS_PER_SEC = 1 000 * MILLIS
    // CLOCKS = 1 000 * MILLIS * CLOCKS_PER_SEC
    return 10 * set->last_plausability > CLOCKS_PER_SEC;
}

void sensor_set_update(sensor_set *set, clock_t now) {
    sensor_update(&set->sensors[0]);
    sensor_update(&set->sensors[1]);

    if (!sensor_set_over_max_implausability_time(set)) {
        // ERROR: Implausability occurred for too long.
        return;
        // exit(1);
    }

    float deviation = calculate_deviation(set);

    // TEMP: MAX_APPS_DEVIATION/MAX_TPS_DEVIATION
    if (deviation < MAX_APPS_DEVIATION) {
        set->last_plausability = now;
    }
}

void update(sensor_set *, sensor_set *);

void main(void) {
    // Initialize the device
    SYSTEM_Initialize();

    //INTERRUPT_PeripheralInterruptEnable();

    sensor_data apps_sensors[2] = {
        {
            .channel = APPS1,
            .minValue = 200,
            .maxValue = 400,
        },
        {
            .channel = APPS2,
            .minValue = 200,
            .maxValue = 400,
        },
    };
    
    sensor_data tps_sensors[2] = {
        {
            .channel = TPS1,
        },
        {
            .channel = TPS2,
        },
    };

    sensor_set apps;
    apps.sensors[0] = apps_sensors[0];
    apps.sensors[1] = apps_sensors[1];

    sensor_set tps;
    tps.sensors[0] = tps_sensors[0];
    tps.sensors[1] = tps_sensors[1];

    while (1) {
        update(&apps, &tps);
        __delay_ms(10);
    }
}

float calculate_deviation(sensor_set *set) {
    float s1 = set->sensors[0].value;
    float s2 = set->sensors[1].value;
    
    // Maybe 2 * this, because we have to take the average
    // between the two values.
    return fabs(s1 - s2) / (s1 + s2);
}

void update(sensor_set *apps, sensor_set *tps) {
//    clock_t now = clock();
    clock_t now = 0;
    sensor_set_update(apps, now);
    sensor_set_update(tps, now);
    
    variable = apps->sensors[1].value;
    return;
}