#include "sample_code.h"


// basicLH
void	sample_Accel_Motion_Capture(sensor_h sensor, sensor_event_s *event, void *data)
{
	static int bMinimum = 0;
	TIMER_CB_DATA*	tcbd	= (TIMER_CB_DATA*)data;
	float*			buffer	= (float*)tcbd->accelBuf;

	if( bMinimum==0 && tcbd->accelPos >= MOTION_LENGTH)
	{
		bMinimum = 1;
	}

	if(tcbd->accelPos == (ACCEL_BUFFER_SIZE+MOTION_LENGTH))
	{
		memcpy(buffer, buffer+(tcbd->accelPos-MOTION_LENGTH)*3, MOTION_LENGTH*3*sizeof(float));
		tcbd->accelPos = MOTION_LENGTH;
	}

	memcpy(buffer+tcbd->accelPos*3+0, event->values+0, sizeof(float));
	memcpy(buffer+tcbd->accelPos*3+1, event->values+1, sizeof(float));
	memcpy(buffer+tcbd->accelPos*3+2, event->values+2, sizeof(float));
	tcbd->accelPos++;

	if( bMinimum!=0 )
	{
		// feature extract
		int i=0, N=0;
		float sumX=0.0f, sumY=0.0f, sumZ=0.0f, sumF=0.0f;
		float sumXY=0.0f, sumXZ=0.0f, sumYZ=0.0f;
		float sumXX=0.0f, sumYY=0.0f, sumZZ=0.0f, sumFF=0.0f;

		// calculate sum
		for(i=0; i<MOTION_LENGTH; i++)
		{
			float x = buffer[(tcbd->accelPos-MOTION_LENGTH+i)*3+0];
			float y = buffer[(tcbd->accelPos-MOTION_LENGTH+i)*3+1];
			float z = buffer[(tcbd->accelPos-MOTION_LENGTH+i)*3+2];
			float xx = pow(x,2.0);
			float yy = pow(y,2.0);
			float zz = pow(z,2.0);
			float f = sqrt(xx+yy+zz);

			N++;
			sumX += x;
			sumY += y;
			sumZ += z;
			sumXY += x*y;
			sumXZ += x*z;
			sumYZ += y*z;
			sumF += f;

			sumXX += xx;
			sumYY += yy;
			sumZZ += zz;
			sumFF += pow(f,2.0);
		}

		// x, y, z sum
		// xy, xz, yz sum
		// xx, yy, zz, ff sum
		if(N==7 || N==8)
		{
			// calculate standard deviation
			float varY, varZ, varF, correlYZ;
			// y standard deviation
			varY = (sumYY/N)-pow(sumY/N,2.0);
			varZ = (sumZZ/N)-pow(sumZ/N,2.0);
			varF = (sumFF/N)-pow(sumF/N,2.0);
			correlYZ = (sumYZ/N -sumY/N * sumZ/N) / (sqrt(varY)*sqrt(varZ));

			// our sample result
					// a y stdev	> 1.562996
			if(		sqrt(varY*N/(N-1)) > 1.562996 &&
					// a f stdev	> 5.808768
					sqrt(varF*N/(N-1)) > 5.808768 &&
					// a yz correl	<= -0.14438
					correlYZ <= -0.14438)
			{
				const long int duration = RECORDING_DURATION;
				tcbd->onMotionInterval = time(0)+duration;
				elm_image_file_set(tcbd->circle_objects->a, G_CIRCLE_FILENAME, 0);
			}
		}
	}
}
int		sample_Audio_Preprocessing(void *data, AUDIO_DATA* audio_seg)
{
	TIMER_CB_DATA *tcbd = (TIMER_CB_DATA*)data;
	void* tSave = 0;
	const int audio_bds = TEMP_AUDIO_BUFFER_DURATION*SAMPLING_RATE*2;
	if(tcbd->bufLen >= audio_bds)
	{
		tSave = malloc(audio_bds);
		memcpy(tSave, tcbd->buffer, audio_bds);
		memmove(tcbd->buffer, tcbd->buffer+audio_bds, tcbd->bufLen-audio_bds);
		tcbd->bufLen -= audio_bds;
	}

	int segCnt = 0;
	if(tSave)
	{
		int i=0, start=-1, end=-1;
		for(i=0; i<TEMP_AUDIO_BUFFER_DURATION*SAMPLING_RATE; i++)
		{
			short temp = *((short*)(tSave+i*2));

			if(temp > IGNORE_RANGE || temp < -IGNORE_RANGE)
			{
				end = -1;
				if(start == -1) start = i; // start hearing some sound.
			}
			else if(start != -1) // On hearing, but no sound.
			{
				if(end == -1)	end = start;	// start counting silence
				else			end++;			// counting silence

				// keep silence. SILENCE_CHECK_DURATION_SAMPLE count.
				// SILENCE_CHECK_DURATION milliseconds.
				if(end-start >= SILENCE_CHECK_DURATION_SAMPLE-1)
				{
					end = i-SILENCE_CHECK_DURATION_SAMPLE;
					// start ~ end 저장
					AUDIO_DATA_init(0, (end-start+1)*2, tSave+start*2, &(audio_seg[segCnt]));
					segCnt++;
					start = -1;
					end = -1;
				}
				// end of data now handled.
				else if(i==TEMP_AUDIO_BUFFER_DURATION*SAMPLING_RATE-1)
				{
					end = i;
					// start ~ end 저장
					AUDIO_DATA_init(0, (end-start+1)*2, tSave+start*2, &(audio_seg[segCnt]));
					segCnt++;
					start = -1;
					end = -1;
				}
			}
		}
		free(tSave);
		tSave = 0;
	}
	return segCnt;
}
void	sample_Speaker_Recognition(void *data, AUDIO_DATA* audio_seg, int segCnt)
{
	TIMER_CB_DATA *tcbd = (TIMER_CB_DATA*)data;
	int i=0;
	for(i=0; i<segCnt; i++)
	{
		AUDIO_DATA *pAudio = audio_seg+i;

		int sum = 0;
		int max = 0;
		unsigned int iter=0;
		for(iter=0; iter<pAudio->length/2; iter++)
		{
			short temp = *((short*)(pAudio->pData+iter*2));
			if( temp < 0 )
			{
				sum += -temp;
				if(-temp > max)
				{
					max = -temp;
				}
			}
			else
			{
				sum += temp;
				if(temp > max)
				{
					max = temp;
				}
			}
		}
		double avg = (double)sum/(double)(pAudio->length/2);
		if(max < 500 || max > 2000 || avg > 300.0)
		{
			AUDIO_DATA_release(audio_seg+i);
			continue;
		}

		double *mfccRes = 0;
		int		nRes = 0;

		if(mfcc(pAudio->pData, pAudio->length, &mfccRes, &nRes) == 0)
		{
			AUDIO_DATA_release(audio_seg+i);
			if(mfccRes)
			{
				free(mfccRes);
				mfccRes=0;
			}
			continue;
		}

		int res = 1;

		int j;
		int count = 0;
		for(j=0; j<nRes; j++)
		{
			int temp =0;
			temp = decision(mfccRes[j*MEL_CEPS_NUM+0], mfccRes[j*MEL_CEPS_NUM+1],mfccRes[j*MEL_CEPS_NUM+2], mfccRes[j*MEL_CEPS_NUM+3],mfccRes[j*MEL_CEPS_NUM+4], mfccRes[j*MEL_CEPS_NUM+5],mfccRes[j*MEL_CEPS_NUM+6], mfccRes[j*MEL_CEPS_NUM+7],mfccRes[j*MEL_CEPS_NUM+8],mfccRes[j*MEL_CEPS_NUM+9],mfccRes[j*MEL_CEPS_NUM+10],mfccRes[j*MEL_CEPS_NUM+11]);
			if(temp==0)
			{
				count++;
			}
		}

		if(nRes/2 < count)
		{
			res = 0;
		}

		if(res == 0)
		{
			const long int duration = 5;
			tcbd->onSpeakerInterval = time(0)+duration;
			elm_image_file_set(tcbd->circle_objects->b, G_CIRCLE_FILENAME, 0);
			dlog_print(DLOG_VERBOSE, "Voice", "My Voice Detected");
		}

		// Saving sound
		time_t reco = time(0);
		struct tm *lt = localtime(&reco);
		if(res==0 && tcbd->onMotionInterval > reco)
		{
			char fn[32];
			sprintf(fn, "/opt/usr/media/Others/%02d%02d%02d.wav", lt->tm_hour, lt->tm_min, lt->tm_sec);
			write_pcm_to_wavFile(fn, SAMPLING_RATE, 16, 1, pAudio->pData, pAudio->length);
		}

		AUDIO_DATA_release(audio_seg+i);
		free(mfccRes);
		mfccRes = 0;
	}
}


// mfcc
#ifndef MEL_FRM_SAMPLE
#define MEL_FRM_SAMPLE MEL_FRM_LEN*SAMPLING_RATE/1000
#endif
COMPLEX COMPLEX_new(double real, double imaginary)
{
	COMPLEX temp={real, imaginary};
	return temp;
}
COMPLEX COMPLEX_add(COMPLEX a, COMPLEX b)
{
	COMPLEX temp={(a.real+b.real),(a.imaginary+b.imaginary)};
	return temp;
}
COMPLEX COMPLEX_sub(COMPLEX a, COMPLEX b)
{
	COMPLEX temp={(a.real-b.real),(a.imaginary-b.imaginary)};
	return temp;
}
COMPLEX COMPLEX_multiply(COMPLEX a, COMPLEX b)
{
	COMPLEX temp={(a.real*b.real-a.imaginary*b.imaginary), (a.real*b.imaginary+a.imaginary*b.real)};
	return temp;
}
int mfcc(const void *input, unsigned int length, double **result, int *nRes)
{
	int i, j, cnt=0;
	short*			pSample = (short*)input;
	unsigned int	nSample = length/2;

	if(*result!=0 || nRes==0)
	{
		return 0;
	}
	if(MEL_FRM_SAMPLE > nSample)
	{
		return 0;
	}

	double			FiltWeight[MEL_FILT_NUM][MEL_FFT_LEN/2+1];

	*nRes = ( (nSample - MEL_FRM_SAMPLE) / (MEL_FRM_SPACE*SAMPLING_RATE/1000) ) + 1;

	*result = (double*)malloc(sizeof(double)*(*nRes)*MEL_CEPS_NUM);

	// init filter
	memset(FiltWeight, 0, sizeof(double)*(MEL_FILT_NUM)*(MEL_FFT_LEN/2+1));
	CreateFilt(FiltWeight);

	// const
	double* Hamming = (double*)malloc(sizeof(double)*MEL_FRM_SAMPLE);
	for(j=0; j<MEL_FRM_SAMPLE; j++)
	{
		Hamming[j] = 0.54-0.46*cos( (double)j*2.0*MEL_PI / (double)(MEL_FRM_SAMPLE-1.0) );
	}
	const double sqrtMelFiltNum = sqrt(2.0/(double)MEL_FILT_NUM);
	double cepsCosValue[MEL_CEPS_NUM][MEL_FILT_NUM];
	for(i=0; i<MEL_CEPS_NUM; i++)
	{
		for(j=0; j<MEL_FILT_NUM; j++)
		{
			cepsCosValue[i][j] = cos(MEL_PI*((double)i+1.0)/((double)MEL_FILT_NUM)*((double)j+0.5));
		}
	}

	for(i=0; i<nSample; i+=MEL_FRM_SPACE*SAMPLING_RATE/1000)
	{
		// framing
		if(i+MEL_FRM_SAMPLE > nSample)
		{
			break;
		}

		short* shData = (short*)malloc(sizeof(short)*MEL_FRM_SAMPLE);
		memcpy(shData, pSample+i, sizeof(short)*MEL_FRM_SAMPLE);

		// hamming window
		double* data = (double*)malloc(sizeof(double)*MEL_FRM_SAMPLE);
		for(j=0; j<MEL_FRM_SAMPLE; j++)
		{
			data[j] = (double)shData[j]*Hamming[j];
		}
		free(shData);
		shData = 0;

		// fft
		COMPLEX* zero_padded = (COMPLEX*)malloc(sizeof(COMPLEX)*MEL_FFT_LEN);
		zero_fft(data,zero_padded);
		free(data);
		data = 0;

		// get fft_mag
		double* fft_mag = (double*)malloc(sizeof(double)*(MEL_FFT_LEN/2+1));
		for(j=0;j<MEL_FFT_LEN/2+1;j++)
		{
			fft_mag[j] = zero_padded[j].real*zero_padded[j].real+zero_padded[j].imaginary*zero_padded[j].imaginary;
		}
		free(zero_padded);
		zero_padded = 0;

		// mel energy
		double* mel_energy = (double*)malloc(sizeof(double)*MEL_FILT_NUM);
		memset(mel_energy, 0, sizeof(double)*MEL_FILT_NUM);
		for(j=0; j<MEL_FILT_NUM; j++)
		{
			int k=0;
			for(k=0; k<MEL_FFT_LEN/2+1; k++)
			{
				mel_energy[j] += FiltWeight[j][k]*fft_mag[k];
			}
			mel_energy[j]=(double)(log(mel_energy[j]));
		}
		free(fft_mag);
		fft_mag = 0;

		// cepstrum
		double* Cep = (double*)malloc(sizeof(double)*MEL_CEPS_NUM);
		for(j=0; j<MEL_CEPS_NUM; j++)
		{
			Cep[j]=0.0;
			int k=0;
			for(k=0; k<MEL_FILT_NUM; k++)
			{
				Cep[j] += mel_energy[k]*cepsCosValue[j][k];
			}
			Cep[j]=sqrtMelFiltNum*Cep[j];
		}
		free(mel_energy);
		mel_energy = 0;

		memcpy((*result)+cnt*MEL_CEPS_NUM, Cep, sizeof(double)*MEL_CEPS_NUM);
		cnt++;
		free(Cep);
		Cep = 0;
	}
	free(Hamming);
	Hamming = 0;
	return 1;
}
void CreateFilt(double (*w)[MEL_FFT_LEN/2+1])
{
   double df=(double)SAMPLING_RATE/(double)MEL_FFT_LEN; // FFT interval
   int indexlow = 0;
   int indexhigh = MEL_round((double)MEL_FFT_LEN/2.0);

   double melmax = 2595.0*log10(1.0+(double)SAMPLING_RATE/1400.0); // mel high frequency
   double melmin = 0.0;
   double melinc=(melmax-melmin)/(double)(MEL_FILT_NUM+1); //mel half bandwidth

   double*	melcenters = (double*)malloc(sizeof(double)*MEL_FILT_NUM);
   double*	fcenters = (double*)malloc(sizeof(double)*MEL_FILT_NUM);
   int*		indexcenter = (int*)malloc(sizeof(int)*MEL_FILT_NUM);
   int*		indexstart = (int*)malloc(sizeof(int)*MEL_FILT_NUM);
   int*		indexstop = (int*)malloc(sizeof(int)*MEL_FILT_NUM);

   double increment,decrement; // increment and decrement of the left and right ramp
   double sum=0.0;
   int i,j;
   for(i=0;i<MEL_FILT_NUM;i++)
   {
		 melcenters[i]=(double)(i+1)*melinc+melmin;   // compute mel center frequencies
		 fcenters[i]=700.0*(pow(10.0,melcenters[i]/2595.0)-1.0); // compute Hertz center frequencies
		 indexcenter[i]=MEL_round(fcenters[i]/df); // compute fft index for Hertz centers
   }
   for(i=0;i<MEL_FILT_NUM-1;i++)  // Compute the start and end FFT index of each channel
	{
		indexstart[i+1]=indexcenter[i];
		indexstop[i]=indexcenter[i+1];
	}
   indexstart[0]=indexlow;
   indexstop[MEL_FILT_NUM-1]=indexhigh;
   for(i=0;i<MEL_FILT_NUM;i++)
   {
	  increment=1.0/((double) indexcenter[i]-(double)indexstart[i]);
	  for(j=indexstart[i];j<=indexcenter[i];j++)
	  {
		 w[i][j]=((double)j-(double)indexstart[i])*increment;
	  }
	  decrement=1.0/((double) indexstop[i]-(double)indexcenter[i]);    // right ramp
	  for(j=indexcenter[i];j<=indexstop[i];j++)
	  {
		 w[i][j]=1.0-((double)j-(double)indexcenter[i])*decrement;
	  }
   }

   for(i=1;i<=MEL_FILT_NUM;i++)     // Normalize filter weights by sum
   {
	   for(j=1;j<=MEL_FFT_LEN/2+1;j++)
	   {
		  sum=sum+w[i-1][j-1];
	   }
	   for(j=1;j<=MEL_FFT_LEN/2+1;j++)
	   {
		  w[i-1][j-1]=w[i-1][j-1]/sum;
	   }
	   sum=0.0;
   }

   free(melcenters);
   free(fcenters);
   free(indexcenter);
   free(indexstart);
   free(indexstop);
}
void zero_fft(double *data,COMPLEX *vec)
{
	for(int i=0;i<MEL_FFT_LEN;i++)
	{
		if(i<MEL_FRM_SAMPLE)
		{
			vec[i] = COMPLEX_new(data[i],0);
		}
		else
		{
			vec[i] = COMPLEX_new(0,0);
		}
	}
	FFT(vec);    // Compute FFT
}
void FFT(COMPLEX *vec)
{
	int ulPower = 0;
	int fftlen1 = MEL_FFT_LEN-1;
	while(fftlen1 > 0)
	{
		ulPower++;
		fftlen1=fftlen1/2;
	}
	int ulIndex;
	int ulK;
	int p;
	for(p=0; p<MEL_FFT_LEN; p++)
	{
		ulIndex = 0;
		ulK = 1;
		int j=0;
		for(j=0; j<ulPower; j++)
		{
			ulIndex += ( (p>>(ulPower-j-1)) & 0x01 ) ? ulK : 0;
			ulK *= 2;
		}
		if(ulIndex>p)
		{
			COMPLEX c = vec[p];
			vec[p] = vec[ulIndex];
			vec[ulIndex] = c;
		}
	}
	COMPLEX* vecW = (COMPLEX*)malloc(sizeof(COMPLEX)*MEL_FFT_LEN/2);
	int i=0;
	for(i=0; i<MEL_FFT_LEN/2; i++)
	{
		vecW[i] = COMPLEX_new(cos(2*i*MEL_PI/MEL_FFT_LEN),-1*sin(2*i*MEL_PI/MEL_FFT_LEN));
	}
	int ulGroupLength = 1;
	int ulHalfLength = 0;
	COMPLEX* pc = (COMPLEX*)malloc(sizeof(COMPLEX)*3); // cw, c1, c2
	int b=0;
	for(b=0; b<ulPower; b++)
	{
		ulHalfLength = ulGroupLength;
		ulGroupLength *= 2;
		int j=0;
		for(j=0; j<MEL_FFT_LEN; j+=ulGroupLength)
		{
			int k=0;
			for(k=0; k<ulHalfLength; k++)
			{
				pc[0] = COMPLEX_multiply( vecW[k*MEL_FFT_LEN/ulGroupLength], vec[j + k + ulHalfLength] );
				pc[1] = COMPLEX_add( vec[j + k], pc[0] );
				pc[2] = COMPLEX_sub( vec[j + k], pc[0] );
				vec[j + k] = pc[1];
				vec[j + k + ulHalfLength] = pc[2];
			}
		}
	}
	free(vecW);
	free(pc);
}
#define son 1
#define kim 0
int decision(double m0, double m1, double m2, double m3, double m4, double m5, double m6, double m7, double m8, double m9, double m10, double m11)
{
	if(m3 <= -0.654858)
	{
		if(m6 <= 1.06145)
		{
			return son;
		}
		else
		{
			if(m7 <= 0.507667)
			{
				return kim;
			}
			else
			{
				return son;
			}
		}
	}
	else
	{
		if(m10 <= 1.179915)
		{
			if(m9 <= -0.390999)
			{
				if(m5 <= 1.026154)
				{
					if(m9 <= -1.074949)
					{
						return son;
					}
					else
					{
						if(m8 <= -0.249427)
						{
							return son;
						}
						else
						{
							if(m4 <= -1.105328)
							{
								return son;
							}
							else
							{
								if(m0 <= 16.851961)
								{
									if(m11 <= -0.706102)
									{
										return son;
									}
									else
									{
										if(m2 <= 0.993606)
										{
											return kim;
										}
										else
										{
											if(m6 <= 0.347578)
											{
												if(m5 <= 0.084518)
												{
													if(m1 <= 5.752533)
													{
														return son;
													}
													else
													{
														return kim;
													}
												}
												else
												{
													return kim;
												}
											}
											else
											{
												return son;
											}
										}
									}
								}
								else
								{
									return kim;
								}
							}
						}
					}
				}
				else
				{
					if(m7 <= 0.294251)
					{
						if(m8 <= -0.535438)
						{
							return son;
						}
						else
						{
							return kim;
						}
					}
					else
					{
						return son;
					}
				}
			}
			else
			{
				if(m6 <= 1.825197)
				{
					if(m7 <= 0.591299)
					{
						if(m8 <= 0.110194)
						{
							return son;
						}
						else
						{
							if(m3 <= 0.770796)
							{
								if(m11 <= 0.37761)
								{
									if(m8 <= 0.596564)
									{
										return son;
									}
									else
									{
										if(m4 <= -1.009122)
										{
											return son;
										}
										else
										{
											return kim;
										}
									}
								}
								else
								{
									if(m2 <= 3.632632)
									{
										return kim;
									}
									else
									{
										return son;
									}
								}
							}
							else
							{
								return kim;
							}
						}
					}
					else
					{
						if(m1 <= 2.980201)
						{
							if(m9 <= 2.051271)
							{
								return son;
							}
							else
							{
								if(m5 <= 2.465791)
								{
									return kim;
								}
								else
								{
									return son;
								}
							}
						}
						else
						{
							if(m6 <= 0.936966)
							{
								if(m9 <= 0.105455)
								{
									return son;
								}
								else
								{
									if(m1 <= 3.703214)
									{
										if(m5 <= 1.225074)
										{
											if(m5 <= 0.593229)
											{
												if(m2 <= 2.786288)
												{
													return son;
												}
												else
												{
													return kim;
												}
											}
											else
											{
												return kim;
											}
										}
										else
										{
											return son;
										}
									}
									else
									{
										return kim;
									}
								}
							}
							else
							{
								if(m7 <= 0.852863)
								{
									return kim;
								}
								else
								{
									if(m4 <= 0.690797)
									{
										return son;
									}
									else
									{
										return kim;
									}
								}
							}
						}
					}
				}
				else
				{
					if(m2 <= 4.077562)
					{
						if(m7 <= 0.066931)
						{
							return kim;
						}
						else
						{
							if(m2 <= 1.147186)
							{
								return son;
							}
							else
							{
								if(m7 <= 1.143845)
								{
									return kim;
								}
								else
								{
									if(m2 <= 3.825164)
									{
										return kim;
									}
									else
									{
										return son;
									}
								}
							}
						}
					}
					else
					{
						return kim;
					}
				}
			}
		}
		else
		{
			if(m1 <= 2.342659)
			{
				if(m4 <= 2.273381)
				{
					return son;
				}
				else
				{
					return kim;
				}
			}
			else
			{
				if(m3 <= 1.087093)
				{
					if(m6 <= -0.297622)
					{
						if(m0 <= 14.309179)
						{
							return kim;
						}
						else
						{
							return son;
						}
					}
					else
					{
						return kim;
					}
				}
				else
				{
					if(m4 <= 1.326693)
					{
						if(m7 <= -1.165319)
						{
							if(m1 <= 3.633522)
							{
								return kim;
							}
							else
							{
								return son;
							}
						}
						else
						{
							if(m4 <= 1.251246)
							{
								if(m8 <= 1.87382)
								{
									return kim;
								}
								else
								{
									return son;
								}
							}
							else
							{
								return son;
							}
						}
					}
					else
					{
						if(m7 <= -0.782368)
						{
							return kim;
						}
						else
						{
							if(m2 <= 2.28768)
							{
								return son;
							}
							else
							{
								return kim;
							}
						}
					}
				}
			}
		}
	}
	return 2;
}


// kmeans
double getDistance(double* a, double* b)
{
	double distance = 0.0;
	int i;

	for(i=0; i<DIMENSIONS; i++)
	{
		distance += pow(b[i]-a[i], 2.0);
	}

	distance = pow(distance, 1.0 / 2.0);

	return distance;
}
double** getInitialCentroids(int k , double** data, int numTuples)
{
	int i, j;
	double *new_centroid_values, **new_centroid;

	new_centroid_values = malloc(DIMENSIONS*k*sizeof(double));
	new_centroid = malloc(k * sizeof(double*));

	for(i=0; i<k; i++)
	{
		new_centroid[i] = new_centroid_values + i*DIMENSIONS;
		for(j=0; j<DIMENSIONS; j++)
		{
			new_centroid[i][j] = data[i][j];
		}
	}

	return new_centroid;
}
int* getNearestCentroids(int k, double** centroid, double** data, int numTuples)
{
	int i, j;
	int indexClosestMean, *newNearestMeans;

	newNearestMeans = malloc(numTuples * sizeof(int));
	for(i=0; i<numTuples; ++i)
	{
		indexClosestMean = 0;
		for(j=1; j<k; ++j)
		{
			if( getDistance(centroid[j], data[i]) < getDistance(centroid[indexClosestMean], data[i]) )
			{
				indexClosestMean = j;
			}
		}
		newNearestMeans[i] = indexClosestMean;
	}

	return newNearestMeans;
}
double** getNewCentroids(int k, int* nearestMeans, double** data, int numTuples)
{
	int i, j, l;
	double *new_centroid_values, **new_centroid;
	double sum[DIMENSIONS];
	int clusterSize = 0;

	memset(sum, 0, sizeof(double)*DIMENSIONS);
	new_centroid_values = malloc(DIMENSIONS*k*sizeof(double));
	new_centroid = malloc(k*sizeof(double*));

	for(i=0; i<k; ++i)
	{
		new_centroid[i] = new_centroid_values + i*DIMENSIONS;

		for(j=0; j<numTuples; ++j)
		{
			if(nearestMeans[j] == i)
			{
				for(l=0; l<DIMENSIONS; l++)
				{
					sum[l] += data[j][l];
				}
				++clusterSize;
			}
		}

		for(l=0; l<DIMENSIONS; l++)
		{
			sum[l] = sum[l] / clusterSize;
			new_centroid[i][l] = sum[l];
			sum[l] = 0.0;
		}
		clusterSize = 0;
	}

	return new_centroid;
}
int compareNearestCentroids(int* last, int* current, int numTuples)
{
	int i;

	if(last==0 || current==0)
	{
		return 0;
	}

	for(i=0; i<numTuples; ++i)
	{
		if(last[i] != current[i])
		{
			return 0;
		}
	}

	return 1;
}
int* kmeans(int k, double** data, int numTuples)
{
	int *nearestMeans, *lastMeans;
	int convergence;
	double **centroid;

	lastMeans = malloc(numTuples*sizeof(int));
	memset(lastMeans, 0, numTuples*sizeof(int));

	convergence = 0;
	// centroid[k][DIMENSIONS]
	centroid = getInitialCentroids(k, data, numTuples);

	do
	{
		// nearestMeans[numTuples]
		nearestMeans = getNearestCentroids(k, centroid, data, numTuples);

		if(centroid)
		{
			if(centroid[0])
			{
				free(centroid[0]);
				centroid[0]=0;
			}
			free(centroid);
			centroid=0;
		}
		// centroid[k][DIMENSIONS]
		centroid = getNewCentroids(k, nearestMeans, data, numTuples);
		convergence = compareNearestCentroids(lastMeans, nearestMeans, numTuples);
		memcpy(lastMeans, nearestMeans, numTuples*sizeof(int));
		if(nearestMeans)
		{
			free(nearestMeans);
			nearestMeans=0;
		}
	}while(convergence != 1);

	if(centroid)
	{
		if(centroid[0])
		{
			free(centroid[0]);
			centroid[0]=0;
		}
		free(centroid);
		centroid=0;
	}

	// lastMeans[numTuples]
	return lastMeans;
}


// saving audio
void write_pcm_to_wavFile(const char* filename, int samplerate, int bitrate, int channel, void* data, int dLen)
{
	int fd = open(filename, O_CREAT|O_WRONLY, 0666);

	char			ChunkID[]		= "RIFF";
	unsigned int	ChunkSize		= dLen + 36;
	char			Format[]		= "WAVE";
	char			Subchunk1ID[]	= "fmt ";
	unsigned int	Subchunk1Size	= 16;
	unsigned short	AudioFormat		= 1;
	unsigned short	NumChannels		= channel;
	unsigned int	SampleRate		= samplerate;
	unsigned int	ByteRate		= SampleRate * NumChannels * bitrate / 8;
	unsigned short	BlockAlign		= NumChannels * bitrate / 8;
	unsigned short	BitsPerSample	= bitrate;
	char			Subchunk2ID[]	= "data";
	unsigned int	Subchunk2Size	= dLen;

	write(fd, ChunkID, 4);
	write(fd, &ChunkSize, 4);
	write(fd, Format, 4);
	write(fd, Subchunk1ID, 4);
	write(fd, &Subchunk1Size, 4);
	write(fd, &AudioFormat, 2);
	write(fd, &NumChannels, 2);
	write(fd, &SampleRate, 4);
	write(fd, &ByteRate, 4);
	write(fd, &BlockAlign, 2);
	write(fd, &BitsPerSample, 2);
	write(fd, Subchunk2ID, 4);
	write(fd, &Subchunk2Size, 4);
	write(fd, data, dLen);

	close(fd);
}
