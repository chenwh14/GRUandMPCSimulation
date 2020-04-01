#define S_FUNCTION_NAME ReferencePreviewer
#define S_FUNCTION_LEVEL 2
//#define MATLAB_MEX_FILE
#include "simstruc.h"
#include <math.h>

#define PARAM_TS ssGetSFcnParam(S, 0)
#define PARAM_LENGTH ssGetSFcnParam(S, 1)

#define MDL_INITIALIZE_CONDITIONS

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 2);
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
    if (!ssSetNumInputPorts(S, 1))
        return;
    if (!ssSetNumOutputPorts(S, 2))
        return;

    ssSetInputPortWidth(S, 0, 1); //value input

    ssSetOutputPortWidth(S, 0, 1); //first
    
    const double *pLength = mxGetPr(PARAM_LENGTH);
    
    ssSetOutputPortWidth(S, 1, round(*pLength)); //seq

    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetNumPWork(S, 2);
    ssSetNumRWork(S, 0);
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
    const double *pLength = mxGetPr(PARAM_LENGTH);

    double* buffer=new double[round(*pLength)+1]{};
    void **pwork = ssGetPWork(S);
    pwork[0] = (void *)buffer;
}

/* Function: mdlOutputs ======================================================
 */

inline void slide(double*input, int size)
{
    
}
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const double *pTs = mxGetPr(PARAM_TS);
    const double *pLength = mxGetPr(PARAM_LENGTH);
    InputRealPtrsType uPtrInput = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *pFirstOut = ssGetOutputPortRealSignal(S, 0);
    real_T *pSeqOut = ssGetOutputPortRealSignal(S, 1);
    void **pwork = ssGetPWork(S);
    double* buffer=(double*)(pwork[0]);
    int length = round(*pLength);
    
    for(int i=0;i<length;i++)
    {
        buffer[i]=buffer[i+1];
    }
    buffer[length]=*uPtrInput[0];
    for(int i=0;i<length;i++)
    {
        pSeqOut[i]=buffer[i+1];
    }
    *pFirstOut=buffer[0];
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    void **pwork = ssGetPWork(S);
    double* buffer=(double*)(pwork[0]);
    delete[] buffer;
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
