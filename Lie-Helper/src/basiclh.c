#include "basiclh.h"
#include "sample_code.h"

// /* Audio Struct
void AUDIO_DATA_release(AUDIO_DATA *in)
{
	if(in==0)
	{
		return;
	}
	if(in->pData != 0)
	{
		free(in->pData);
		in->pData = 0;
	}
	in->length = 0;
	in->time = 0;
}
void AUDIO_DATA_init(time_t time, unsigned int length, const void* data, AUDIO_DATA* in)
{
	//AUDIO_DATA_release(in);
	if(in == 0 || data == 0)
	{
		return;
	}
	in->time = time;
	in->length = length;
	in->pData = malloc(length);
	memcpy(in->pData, data, length);
}
// Audio Struct */

typedef struct appdata {
	Evas_Object *win;
	Evas_Object *conform;
	Evas_Object *label;
} appdata_s;

static void
win_delete_request_cb(void *data, Evas_Object *obj, void *event_info)
{
	ui_app_exit();
}

static void
win_back_cb(void *data, Evas_Object *obj, void *event_info)
{
	appdata_s *ad = data;
	/* Let window go to hide state. */
	elm_win_lower(ad->win);
}

static void Gyro_cb(sensor_h sensor, sensor_event_s *event, void *data)
{
	// gyroscope data
	// you can use argument data as TIMER_CB_DATA to share sensor data.

	/*
	 *  Your code should be here
	 *  for Motion Capture using gyroscope.
	 *  Our Sample do not using this data.
	 */
}

static void Accel_cb(sensor_h sensor, sensor_event_s *event, void *data)
{
	// accelerometer data

	/*
	 *  Your code should be here
	 *  for Motion Capture using gyroscope.
	 */

	sample_Accel_Motion_Capture(sensor, event, data);
}

static Eina_Bool timer_cb(void *data)
{
	//// audio recording
	TIMER_CB_DATA *tcbd = (TIMER_CB_DATA*)data;
	audio_in_h hAudio = tcbd->hAudio;
	void *buffer = malloc(SAMPLING_RATE*2*TIMER_PERIOD);
	int readByte = 0;
	readByte = audio_in_read(hAudio, buffer, SAMPLING_RATE*2*TIMER_PERIOD);
	memcpy(tcbd->buffer+tcbd->bufLen, buffer, readByte);
	tcbd->bufLen += readByte;
	free(buffer);


	/*
	 *  Your code should be here
	 *  for Audio preproscessing.
	 */
	AUDIO_DATA audio_seg[1000*TEMP_AUDIO_BUFFER_DURATION/SILENCE_CHECK_DURATION];
	int segCnt;
	segCnt = sample_Audio_Preprocessing(data, audio_seg);


	/*
	 *  Your code should be here
	 *  for Speaker recognition and saving voice.
	 */
	sample_Speaker_Recognition(data, audio_seg, segCnt);


	// sample ui
	if(tcbd->onMotionInterval!=0 && tcbd->onMotionInterval < time(0))
	{
		elm_image_file_set(tcbd->circle_objects->a, R_CIRCLE_FILENAME, 0);
		tcbd->onMotionInterval = 0;
	}
	if(tcbd->onSpeakerInterval!=0 && tcbd->onSpeakerInterval < time(0))
	{
		elm_image_file_set(tcbd->circle_objects->b, R_CIRCLE_FILENAME, 0);
		tcbd->onSpeakerInterval = 0;
	}

	return EINA_TRUE;
}

static void button_clicked_cb(void* data, Evas_Object *obj, void *event_info)
{
	static bool isRunning = false;
	static sensor_h sGyro = 0;
	static sensor_listener_h lGyro = 0;
	static sensor_h sAccel = 0;
	static sensor_listener_h lAccel = 0;
	static audio_in_h hAudio = 0;
	static Ecore_Timer* timer = 0;
	static TIMER_CB_DATA tcbd;
	tcbd.circle_objects = (TWO_CIRCLE*)data;

	if(sGyro == 0) // create gyroscope
	{
		if(sensor_get_default_sensor(SENSOR_GYROSCOPE, &sGyro) != SENSOR_ERROR_NONE)
		{
			sGyro = 0;
			return;
		}
		if(sensor_create_listener(sGyro, &lGyro) != SENSOR_ERROR_NONE)
		{
			sGyro = 0;
			lGyro = 0;
			return;
		}
		sensor_listener_set_option(lGyro, SENSOR_OPTION_ALWAYS_ON);
	}

	if(sAccel == 0) // create accelerometer
	{
		if(sensor_get_default_sensor(SENSOR_LINEAR_ACCELERATION , &sAccel) != SENSOR_ERROR_NONE)
		{
			sAccel = 0;
			return;
		}
		if(sensor_create_listener(sAccel, &lAccel) != SENSOR_ERROR_NONE)
		{
			sAccel = 0;
			lAccel = 0;
			return;
		}
		sensor_listener_set_option(lAccel, SENSOR_OPTION_ALWAYS_ON);
	}

	if(isRunning)
	{// run -> stop
		// stop timer
		if(timer)
		{
			ecore_timer_del(timer);
			timer = 0;
			tcbd.hAudio = hAudio;
			tcbd.bufLen = 0;
			if(tcbd.buffer)
			{
				free(tcbd.buffer);
				tcbd.buffer = 0;
			}
		}
		// stop audio recording
		audio_in_unprepare(hAudio);
		audio_in_destroy(hAudio);

		// stop gyroscope
		sensor_listener_stop(lGyro);
		sensor_listener_unset_event_cb(lGyro);

		// stop accelerometer
		sensor_listener_stop(lAccel);
		sensor_listener_unset_event_cb(lAccel);
		tcbd.onMotionInterval = 0;
		elm_image_file_set(tcbd.circle_objects->a, R_CIRCLE_FILENAME, 0);
		tcbd.onSpeakerInterval = 0;
		elm_image_file_set(tcbd.circle_objects->b, R_CIRCLE_FILENAME, 0);
		tcbd.accelPos = 0;
		if(tcbd.accelBuf)
		{
			free(tcbd.accelBuf);
			tcbd.accelBuf = 0;
		}

		elm_object_text_set(obj, "Start");
		isRunning = false;
	}
	else
	{// stop->run
		// run gyroscope
		sensor_listener_set_event_cb(lGyro, 0, Gyro_cb, &tcbd);
		sensor_listener_start(lGyro);

		// run accelerometer
		tcbd.onMotionInterval = 0;
		tcbd.onSpeakerInterval = 0;
		tcbd.accelPos = 0;

		// xyz -> *3, reserve -> *2
		tcbd.accelBuf = malloc(ACCEL_BUFFER_SIZE*3*sizeof(float)*2);
		sensor_listener_set_event_cb(lAccel, 0, Accel_cb, &tcbd);
		sensor_listener_start(lAccel);

		// run audio recording
		audio_in_create(SAMPLING_RATE, AUDIO_CHANNEL_MONO, AUDIO_SAMPLE_TYPE_S16_LE, &hAudio);
		audio_in_prepare(hAudio);

		// run timer
		if(timer == 0)
		{
			tcbd.hAudio = hAudio;
			tcbd.bufLen = 0;
			tcbd.buffer = malloc(SAMPLING_RATE*SAVE_BUFFER_SIZE*sizeof(short));
			elm_image_file_set(tcbd.circle_objects->a, R_CIRCLE_FILENAME, 0);
			elm_image_file_set(tcbd.circle_objects->b, R_CIRCLE_FILENAME, 0);

			timer = ecore_timer_add(TIMER_PERIOD, timer_cb, &tcbd);
		}

		elm_object_text_set(obj, "Stop");
		isRunning = true;
	}
}

static void
create_base_gui(appdata_s *ad)
{
	/* Window */
	ad->win = elm_win_util_standard_add(PACKAGE, PACKAGE);
	elm_win_autodel_set(ad->win, EINA_TRUE);

	if (elm_win_wm_rotation_supported_get(ad->win)) {
		int rots[4] = { 0, 90, 180, 270 };
		elm_win_wm_rotation_available_rotations_set(ad->win, (const int *)(&rots), 4);
	}

	evas_object_smart_callback_add(ad->win, "delete,request", win_delete_request_cb, NULL);
	eext_object_event_callback_add(ad->win, EEXT_CALLBACK_BACK, win_back_cb, ad);

	/* Conformant */
	ad->conform = elm_conformant_add(ad->win);
	elm_win_indicator_mode_set(ad->win, ELM_WIN_INDICATOR_SHOW);
	elm_win_indicator_opacity_set(ad->win, ELM_WIN_INDICATOR_OPAQUE);
	evas_object_size_hint_weight_set(ad->conform, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);
	elm_win_resize_object_add(ad->win, ad->conform);
	evas_object_show(ad->conform);

	/* custom ui */

	Evas_Object *grid;
	grid = elm_grid_add(ad->conform);
	evas_object_size_hint_weight_set(grid, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);
	elm_grid_size_set(grid, 9, 10);
	elm_object_content_set(ad->conform, grid);
	evas_object_show(grid);

	Evas_Object *img1;
	img1 = elm_image_add(grid);
	elm_image_aspect_fixed_set(img1, EINA_TRUE);
	elm_image_file_set(img1, R_CIRCLE_FILENAME, 0);
	elm_grid_pack(grid, img1, 1, 2, 3, 3);
	evas_object_show(img1);

	Evas_Object *img2;
	img2 = elm_image_add(grid);
	elm_image_aspect_fixed_set(img2, EINA_TRUE);
	elm_image_file_set(img2, R_CIRCLE_FILENAME, 0);
	elm_grid_pack(grid, img2, 5, 2, 3, 3);
	evas_object_show(img2);

	static TWO_CIRCLE circle_data = {0, 0};
	circle_data.a=img1;
	circle_data.b=img2;

	Evas_Object *bt3;
	bt3 = elm_button_add(grid);
	elm_object_text_set(bt3, BUTTON_TEXT_START);
	evas_object_size_hint_weight_set(bt3, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);
	evas_object_size_hint_align_set(bt3, EVAS_HINT_FILL, EVAS_HINT_FILL);
	evas_object_smart_callback_add(bt3, "clicked", button_clicked_cb, &circle_data);
	elm_grid_pack(grid, bt3, 0, 6, 9, 3);
	evas_object_show(bt3);

	/* Show window after base gui is set up */
	evas_object_show(ad->win);
}

static bool
app_create(void *data)
{
	/* Hook to take necessary actions before main event loop starts
		Initialize UI resources and application's data
		If this function returns true, the main loop of application starts
		If this function returns false, the application is terminated */
	appdata_s *ad = data;

	create_base_gui(ad);

	return true;
}

static void
app_control(app_control_h app_control, void *data)
{
	/* Handle the launch request. */
}

static void
app_pause(void *data)
{
	/* Take necessary actions when application becomes invisible. */
    //appdata_s *ad = data;

	// If application is running, display will not be off.
    device_power_release_lock(POWER_LOCK_DISPLAY);

}

static void
app_resume(void *data)
{
	/* Take necessary actions when application becomes visible. */
	//appdata_s *ad = data;

	device_power_request_lock(POWER_LOCK_DISPLAY, 0);
}

static void
app_terminate(void *data)
{
	/* Release all resources. */
}

static void
ui_app_lang_changed(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LANGUAGE_CHANGED*/
	char *locale = NULL;
	system_settings_get_value_string(SYSTEM_SETTINGS_KEY_LOCALE_LANGUAGE, &locale);
	elm_language_set(locale);
	free(locale);
	return;
}

static void
ui_app_orient_changed(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_DEVICE_ORIENTATION_CHANGED*/
	return;
}

static void
ui_app_region_changed(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_REGION_FORMAT_CHANGED*/
}

static void
ui_app_low_battery(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LOW_BATTERY*/
}

static void
ui_app_low_memory(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LOW_MEMORY*/
}

int
main(int argc, char *argv[])
{
	appdata_s ad = {0,};
	int ret = 0;

	ui_app_lifecycle_callback_s event_callback = {0,};
	app_event_handler_h handlers[5] = {NULL, };

	event_callback.create = app_create;
	event_callback.terminate = app_terminate;
	event_callback.pause = app_pause;
	event_callback.resume = app_resume;
	event_callback.app_control = app_control;

	ui_app_add_event_handler(&handlers[APP_EVENT_LOW_BATTERY], APP_EVENT_LOW_BATTERY, ui_app_low_battery, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_LOW_MEMORY], APP_EVENT_LOW_MEMORY, ui_app_low_memory, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_DEVICE_ORIENTATION_CHANGED], APP_EVENT_DEVICE_ORIENTATION_CHANGED, ui_app_orient_changed, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_LANGUAGE_CHANGED], APP_EVENT_LANGUAGE_CHANGED, ui_app_lang_changed, &ad);
	ui_app_add_event_handler(&handlers[APP_EVENT_REGION_FORMAT_CHANGED], APP_EVENT_REGION_FORMAT_CHANGED, ui_app_region_changed, &ad);
	ui_app_remove_event_handler(handlers[APP_EVENT_LOW_MEMORY]);

	ret = ui_app_main(argc, argv, &event_callback, &ad);
	if (ret != APP_ERROR_NONE) {
		dlog_print(DLOG_ERROR, LOG_TAG, "app_main() is failed. err = %d", ret);
	}

	return ret;
}
