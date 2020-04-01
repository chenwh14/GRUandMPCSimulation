#define S_FUNCTION_NAME  MPCwrapper
#define S_FUNCTION_LEVEL 2
//#define MATLAB_MEX_FILE
#include "simstruc.h"
#include "MPController.h"

#define PARAM_TS ssGetSFcnParam(S, 0)
#define PARAM_P ssGetSFcnParam(S, 1)
#define PARAM_M ssGetSFcnParam(S, 2)
#define PARAM_WY ssGetSFcnParam(S, 3)
#define PARAM_WU ssGetSFcnParam(S, 4)
#define PARAM_PN ssGetSFcnParam(S, 5)
#define PARAM_MN ssGetSFcnParam(S, 6)
#define PARAM_A ssGetSFcnParam(S, 7)
#define PARAM_B ssGetSFcnParam(S, 8)
#define PARAM_C ssGetSFcnParam(S, 9)
 
#define MDL_INITIALIZE_CONDITIONS
//#define MDL_UPDATE

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 10);
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
    if (!ssSetNumOutputPorts(S,1)) return;
    
    ssSetInputPortWidth(S,0,1);//mo
    
    const double *pp=mxGetPr(PARAM_P);

    ssSetInputPortWidth(S,1,DYNAMICALLY_SIZED);//ref width is dynamically sized, in case the reference preview length is smaller than p
    
    ssSetOutputPortWidth(S,0,1);//mo

    
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

#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
static void mdlSetInputPortWidth(SimStruct *S, int_T port,
                                    int_T inputPortWidth)
{
    ssSetInputPortWidth(S,port,inputPortWidth);
}
# define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int_T port,
                                    int_T outputPortWidth)
{
    ssSetOutputPortWidth(S,port,outputPortWidth);
}

static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    ssSetInputPortWidth(S, 1, 1);
}
#endif

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
    const double *p=mxGetPr(PARAM_P);
    const double *m=mxGetPr(PARAM_M);
    const double *wy=mxGetPr(PARAM_WY);
    const double *wu=mxGetPr(PARAM_WU);
    const double *pn=mxGetPr(PARAM_PN);
    const double *mn=mxGetPr(PARAM_MN);
    const double *pA=mxGetPr(PARAM_A);
    const double *pB=mxGetPr(PARAM_B);
    const double *pC=mxGetPr(PARAM_C);
    
    size_t        pSize = mxGetNumberOfDimensions(PARAM_A);
    const int  *pDims = mxGetDimensions(PARAM_A);
    int rows=pDims[0],cols=pDims[1];
    int n=rows;
    double *_A=new double[rows*cols];
    for (size_t i=0;i<rows;i++)
        for(size_t j=0;j<cols;j++)
            _A[i*cols+j]=pA[i+rows*j];

    MPController *mpcontroller = new MPController(_A,pB,pC,n,*p,*m,*wy,*wu);
    
    double *_P=new double[n*n];
    double *_Q=new double[n*n];
    for(int i=0;i<n;i++)
        for (int j=0;j<n;j++)
          {
            if(i==j) 
            { 
                _P[i*n+j]=1;
                _Q[i*n+j]=*pn;
            }
            else
            { 
                _P[i*n+j]=0;
                _Q[i*n+j]=0;}  
          }

    mpcontroller->InitKalmanFilter(_P,_Q,*mn);
    void **pwork = ssGetPWork(S);
    pwork[0]=(void *)mpcontroller;
    
}
/*
static void mdlUpdate(SimStruct *S, int_T tid)
{
    //double sum=0;
   // for(int i=0;i<mpcontroller->p*mpcontroller->m;i++)
    //    sum+=((double*)(mpcontroller->Kmpc))[i];
}
*/
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *y    = ssGetOutputPortRealSignal(S,0);

    void **pwork = ssGetPWork(S);

    MPController *mpcontroller=(MPController*)(pwork[0]);

    InputRealPtrsType uPtrMo  = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType uPtrRef  = ssGetInputPortRealSignalPtrs(S,1);
    int_T  refWidth = ssGetInputPortWidth(S,1);


    double mo = *uPtrMo[0];
    double mv = mpcontroller->Update(mo,*uPtrRef,refWidth);
    
    
    y[0]=mv;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    void **pwork = ssGetPWork(S);
    MPController *mpcontroller=(MPController*)(pwork[0]);
    delete mpcontroller;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
