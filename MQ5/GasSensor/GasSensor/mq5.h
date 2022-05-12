#define RL_VALUE (10)     //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR (9.83)  //(Sensor resistance in clean air)/RO,which is derived from the chart in datasheet
#define LPG (0)         // Gas identity no.
#define SMOKE (1)
													                                                 
int  GetPercentage(float, float *);
int  GetPercentagee(float, float , float);
int GetGasPercentage(float , int);
float ReadSensor();
float ResistanceCalculation(int);
float SensorCalibration();
