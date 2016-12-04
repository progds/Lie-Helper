#ifndef __basiclh_H__
#define __basiclh_H__

#include <app.h>
#include <Elementary.h>
#include <system_settings.h>
#include <efl_extension.h>
#include <dlog.h>

//// include header
#include <device/power.h>
#include <unistd.h>
#include <fcntl.h>
#include <sensor.h>
#include <storage.h>
#include <time.h>
#include <audio_io.h>
#include <math.h>
#include<stdlib.h>
#include<string.h>
#include <service_app.h>
//// define value
#define BUTTON_TEXT_START "Start"
#define TIMER_PERIOD 1
#define SAVE_BUFFER_SIZE 100
#define SAMPLING_RATE 44100
#define SILENCE_CHECK_DURATION 500 // milli second
#define SILENCE_CHECK_DURATION_SAMPLE SAMPLING_RATE*SILENCE_CHECK_DURATION/1000
#define TEMP_AUDIO_BUFFER_DURATION 5 // second
#define IGNORE_RANGE 300
#define ACCEL_BUFFER_SIZE 10
#define MOTION_LENGTH 8
#define RECORDING_DURATION 30 // second
#define MINIMUN_FRANE 200

#define R_CIRCLE_FILENAME "/opt/usr/apps/test.pino.basiclh/res/red.png"
#define G_CIRCLE_FILENAME "/opt/usr/apps/test.pino.basiclh/res/green.png"
//// define datatype
// /* audio struct
typedef struct sttAudio
{
	time_t time;
	unsigned int length;
	void *pData;
}AUDIO_DATA; // only include length and binary data.(No data about sample rate, sample bit)
void AUDIO_DATA_release(AUDIO_DATA *in);
void AUDIO_DATA_init(time_t time, unsigned int length, const void* data, AUDIO_DATA* in);
// audio struct */

typedef struct sttTwoCircle {
	Evas_Object *a;
	Evas_Object *b;
}TWO_CIRCLE;

// /* callback data
typedef struct sttTimerCbData
{
	audio_in_h	hAudio;
	FILE*		fp;

	// audio buffer
	void*		buffer;		// short
	int			bufLen;

	// accel buffer
	void*		accelBuf;	// float, size = ACCEL_BUFFER_SIZE*3*2*sizeof(float)
	int			accelPos;	// index of next data position 0~(ACCEL_BUFFER_SIZE*2)

	// state
	TWO_CIRCLE	*circle_objects;
	long int onMotionInterval;
	long int onSpeakerInterval;
}TIMER_CB_DATA;
// callback data */

#ifdef  LOG_TAG
#undef  LOG_TAG
#endif
#define LOG_TAG "basiclh"

#if !defined(PACKAGE)
#define PACKAGE "test.pino.basiclh"
#endif

#endif /* __basiclh_H__ */
