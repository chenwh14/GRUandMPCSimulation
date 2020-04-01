#define S_FUNCTION_NAME ProfileGenerator
#define S_FUNCTION_LEVEL 2
//#define MATLAB_MEX_FILE
#include "simstruc.h"
#include "TrapezoidalPathGen.h"

#define PARAM_TS ssGetSFcnParam(S, 0)
#define PARAM_VMAX ssGetSFcnParam(S, 1)
#define PARAM_AMAX ssGetSFcnParam(S, 2)

#define MDL_INITIALIZE_CONDITIONS
#define MDL_UPDATE

inline void slide(double* array, int size, bool duplicate)
{
    for(int i=0;i<size-1;i++)
    {
        array[i]=array[i+1];
    }
    if(!duplicate)
    {
        array[size-1]=0;
    }
}



/*================*
 * Build checking *
 *================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 3);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    {
        int iParam = 0;
        int nParam = ssGetNumSFcnParams(S);

        for (iParam = 0; iParam < nParam; iParam++)
        {
            ssSetSFcnParamTunable(S, iParam, SS_PRM_NOT_TUNABLE);
        }
    }

    /* Set number of input and output ports */
    if (!ssSetNumInputPorts(S, 3))
        return;
    if (!ssSetNumOutputPorts(S, 2))
        return;

    ssSetInputPortWidth(S, 0, 2); //command input
    ssSetInputPortWidth(S, 1, 1); //current position
    ssSetInputPortWidth(S, 2, 1); //current velocity

    ssSetOutputPortWidth(S, 0, 1); //position target
    ssSetOutputPortWidth(S, 1, 1); //velocity target

    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 1, 0);
    ssSetInputPortDirectFeedThrough(S, 2, 0);

    ssSetNumPWork(S, 2);
    ssSetNumRWork(S, 1);
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                     SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    const double *pTs = mxGetPr(PARAM_TS);
    ssSetSampleTime(S, 0, *pTs);
    ssSetOffsetTime(S, 0, 0.0);
    //ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void mdlInitializeConditions(SimStruct *S)
{
    const double *pTs = mxGetPr(PARAM_TS);
    const double *pVmax = mxGetPr(PARAM_VMAX);
    const double *pAmax = mxGetPr(PARAM_AMAX);

    int bufferSize = round(0.1 / (*pTs));
    double *posBuffer = new double[bufferSize]{};
    double *velBuffer = new double[bufferSize]{};

    void **pwork = ssGetPWork(S);
    pwork[0] = (void *)posBuffer;
    pwork[1] = (void *)velBuffer;
}

static void mdlUpdate(SimStruct *S, int_T tid)
{
    const double *pTs = mxGetPr(PARAM_TS);
    const double *pVmax = mxGetPr(PARAM_VMAX);
    const double *pAmax = mxGetPr(PARAM_AMAX);

    int bufferSize = round(0.1 / (*pTs));
    void **pwork = ssGetPWork(S);

    double *posBuffer = (double *)(pwork[0]);
    double *velBuffer = (double *)(pwork[1]);
    real_T *rwork=ssGetRWork(S);

    InputRealPtrsType uPtrCommand = ssGetInputPortRealSignalPtrs(S, 0);
    InputRealPtrsType uPtrCurPos = ssGetInputPortRealSignalPtrs(S, 1);
    InputRealPtrsType uPtrCurVel = ssGetInputPortRealSignalPtrs(S, 2);

    double command[] = {*uPtrCommand[0], *uPtrCommand[1]};
    double curPos = *uPtrCurPos[0], curVel = *uPtrCurVel[0];

    if(command[0]==0)
    {
        slide(posBuffer,bufferSize,true);
        slide(velBuffer,bufferSize,false);
    }
    else
    {
        TrapezoidalPathGen(curPos,command[1],curVel,*pTs,*pVmax,*pAmax,bufferSize,posBuffer,velBuffer);
    }
    rwork[0]=curVel;
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    void **pwork = ssGetPWork(S);

    double *posBuffer = (double *)(pwork[0]);
    double *velBuffer = (double *)(pwork[1]);

    real_T *xout = ssGetOutputPortRealSignal(S, 0);
    real_T *yout = ssGetOutputPortRealSignal(S, 1);

    xout[0]=posBuffer[0];
    yout[0]=velBuffer[0];

}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    void **pwork = ssGetPWork(S);

    double *posBuffer = (double *)(pwork[0]);
    double *velBuffer = (double *)(pwork[1]);

    delete[] posBuffer;
    delete[] velBuffer;
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
