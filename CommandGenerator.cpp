#define S_FUNCTION_NAME CommandGenerator
#define S_FUNCTION_LEVEL 2
//#define MATLAB_MEX_FILE
#include "simstruc.h"
#include "TrapezoidalPathGen.h"

#define PARAM_TS ssGetSFcnParam(S, 0)
#define PARAM_PERIOD ssGetSFcnParam(S, 1)

/*================*
 * Build checking *
 *================*/

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
    if (!ssSetNumOutputPorts(S, 1))
        return;

    ssSetInputPortWidth(S, 0, 1); //value input

    ssSetOutputPortWidth(S, 0, 2); //command output

    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetNumPWork(S, 0);
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


/* Function: mdlOutputs ======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const double *pTs = mxGetPr(PARAM_TS);
    const double *pPeriod = mxGetPr(PARAM_PERIOD);
    InputRealPtrsType pValue = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *pCommandOut = ssGetOutputPortRealSignal(S, 0);
    real_T *rwork = ssGetRWork(S);

    double t=(double)ssGetT(S);
    
    while(t>=*pPeriod-2.2204e-16)
    {
        t-=*pPeriod;
    }
        
    if((t<*pTs)&&(rwork[0]==0))
    {
        pCommandOut[0]=1;
        pCommandOut[1]=*pValue[0];
        rwork[0]=1;
    }
    else
    {
        pCommandOut[0]=0;
        pCommandOut[1]=0;
        rwork[0]=0;
    }

}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
