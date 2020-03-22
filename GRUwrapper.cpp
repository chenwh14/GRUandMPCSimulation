#define S_FUNCTION_NAME  GRUwrapper
#define S_FUNCTION_LEVEL 2
//#define MATLAB_MEX_FILE
#include "simstruc.h"
#include "GRUPredict.h"

#define PARAM_TS ssGetSFcnParam(S, 0)
 
#define MDL_INITIALIZE_CONDITIONS
#define MDL_UPDATE

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    
    {
    int iParam = 0;
    int nParam = ssGetNumSFcnParams(S);
    
    for ( iParam = 0; iParam < nParam; iParam++ )
       {
        ssSetSFcnParamTunable( S, iParam, SS_PRM_NOT_TUNABLE);
       }
    }
    
    /* Set number of input and output ports */
    if (!ssSetNumInputPorts( S,2)) return;
    if (!ssSetNumOutputPorts(S,2)) return;
    
    ssSetInputPortWidth(S,0,1);//x
    ssSetInputPortWidth(S,1,1);//y
    
    ssSetOutputPortWidth(S,0,6);//x predictive out
    ssSetOutputPortWidth(S,1,6);//y predictive out
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    
    ssSetNumPWork(S,1);
    ssSetNumRWork(S,0);
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
    const double *pTs=mxGetPr(PARAM_TS);
    ssSetSampleTime(S, 0, *pTs);
    ssSetOffsetTime(S, 0, 0.0);
    //ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

static void mdlInitializeConditions(SimStruct *S)
{
    const double *Ts=mxGetPr(PARAM_TS);

    GRUPredict *xyPredictor = new GRUPredict("x.json","y.json");
    
    void **pwork = ssGetPWork(S);
    pwork[0]=(void *)xyPredictor;
}

static void mdlUpdate(SimStruct *S, int_T tid)
{
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    void **pwork = ssGetPWork(S);
    GRUPredict *xyPredictor = (GRUPredict*)(pwork[0]);
    
    InputRealPtrsType uPtrCurX  = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType uPtrCurY  = ssGetInputPortRealSignalPtrs(S,1);

    float* input = new float[2]{(float)*uPtrCurX[0],(float)*uPtrCurY[0]};
    float* outputX = new float[6]{};
    float* outputY = new float[6]{};
    
    xyPredictor->Input(input);
    xyPredictor->Predict(outputX,outputY);
    
    real_T *xout = ssGetOutputPortRealSignal(S,0);
    real_T *yout = ssGetOutputPortRealSignal(S,1);
    
    for (int i=0;i<6;i++)
    {
        xout[i]=outputX[i];
        yout[i]=outputY[i];
    }
    delete[] input;
    delete[] outputX;
    delete[] outputY;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    void **pwork = ssGetPWork(S);
    GRUPredict *xyPredictor = (GRUPredict*)(pwork[0]);
    delete xyPredictor;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
