#ifdef __cplusplus
extern "C" {
#endif

//#include "unity.h"
#include "flexsea-user_test-all.h"

//****************************************************************************
// Variables used for these sets of tests:
//****************************************************************************

//****************************************************************************
// Helper function(s):
//****************************************************************************

//****************************************************************************
// Main test function:
//****************************************************************************

//Call this function to test the 'flexsea-system' stack:
void flexsea_user_test(void)
{
	//One call per file here:
	test_ankle2dof();
}

#ifdef __cplusplus
}
#endif
