/*
 * media_filter.c
 *
 *  Created on: Nov 14, 2019
 *      Author: matt carney
 */




#include "median_filter.h"

#include "user-mn-MIT-DLeg.h"



#define PIX_SWAP(a,b) { float temp=(a);(a)=(b);(b)=temp; }
#define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }

/*
 * Median Filter
 * in: 		Input currently measured value, and pointer to window array, so can be used elsewhere
 * return: 	median value from a rolling window
 */
float medianFilterData3( float *inputData, float *origArray)
{
	float copyArray[MEDIAN_FILTER_WINDOW_SIZE_3];			//establish window to track values
	float filteredOutput =0.0;

	/* shift window forward, i=0 is newest element */
	for(uint8_t i = MEDIAN_FILTER_WINDOW_SIZE_3-1; i > 0; --i)
	{
		origArray[i] = origArray[i-1];	//rebuild dataArray with unsorted data.
	}

	/* Update newest value */
	origArray[0] = *inputData;

	/* Copy the data */
	memcpy(copyArray, origArray, sizeof(copyArray));

	/* Sort it & Take the middle value */
	filteredOutput = opt_med3(copyArray);

	return filteredOutput;
}


/*
 * Median Filter
 * in: 		Input currently measured value
 * return: 	median value from a rolling window
 */
float medianFilterData5( float *inputData, float *origArray )
{
	float copyArray[MEDIAN_FILTER_WINDOW_SIZE_5];			//establish window to track values
	float filteredOutput =0.0;

	/* shift window forward, i=0 is newest element */
	for(uint8_t i = MEDIAN_FILTER_WINDOW_SIZE_5-1; i > 0; --i)
	{
		origArray[i] = origArray[i-1];	//rebuild dataArray with unsorted data.
	}

	/* Update newest value */
	origArray[0] = *inputData;

	/* Copy the data */
	memcpy(copyArray, origArray, sizeof(copyArray));

	/* Sort it & Take the middle value */
	filteredOutput = opt_med5(copyArray);

	return filteredOutput;
}


/*
 * Median Filter
 * in: 		Input currently measured value
 * return: 	median value from a rolling window
 */
float medianFilterData7( float *inputData, float *origArray )
{
	float copyArray[MEDIAN_FILTER_WINDOW_SIZE_7];			//establish window to track values
	float filteredOutput =0.0;

	/* shift window forward, i=0 is newest element */
	for(uint8_t i = MEDIAN_FILTER_WINDOW_SIZE_7-1; i > 0; --i)
	{
		origArray[i] = origArray[i-1];	//rebuild dataArray with unsorted data.
	}

	/* Update newest value */
	origArray[0] = *inputData;

	/* Copy the data */
	memcpy(copyArray, origArray, sizeof(copyArray));

	/* Sort it & Take the middle value */
	filteredOutput = opt_med7(copyArray);

	return filteredOutput;
}

/*
 * Median Filter
 * in: 		Input currently measured value
 * return: 	median value from a rolling window
 */
float medianFilterData9( float *inputData, float *origArray )
{
	float copyArray[MEDIAN_FILTER_WINDOW_SIZE_9];			//establish window to track values
	float filteredOutput =0.0;

	/* shift window forward, i=0 is newest element */
	for(uint8_t i = MEDIAN_FILTER_WINDOW_SIZE_9-1; i > 0; --i)
	{
		origArray[i] = origArray[i-1];	//rebuild dataArray with unsorted data.
	}

	/* Update newest value */
	origArray[0] = *inputData;

	/* Copy the data */
	memcpy(copyArray, origArray, sizeof(copyArray));

	/* Sort it & Take the middle value */
	filteredOutput = opt_med9(copyArray);

	return filteredOutput;
}


/*
 * Median Filter
 * in: 		Input currently measured value
 * return: 	median value from a rolling window
 */
float medianFilterData25( float *inputData, float *origArray)
{
	float copyArray[MEDIAN_FILTER_WINDOW_SIZE_25];			//establish window to track values
	float filteredOutput =0.0;

	/* shift window forward, i=0 is newest element */
	for(uint8_t i = MEDIAN_FILTER_WINDOW_SIZE_25-1; i > 0; --i)
	{
		origArray[i] = origArray[i-1];	//rebuild dataArray with unsorted data.
	}

	/* Update newest value */
	origArray[0] = *inputData;

	/* Copy the data */
	memcpy(copyArray, origArray, sizeof(copyArray));

	/* Sort it & Take the middle value */
	filteredOutput = opt_med25(copyArray);

	return filteredOutput;
}


/*
 * The following routines have been built from knowledge gathered
 * around the Web. I am not aware of any copyright problem with
 * them, so use it as you want.
 * N. Devillard - 1998
 * http://ndevilla.free.fr/median/median/index.html
 */

/*----------------------------------------------------------------------------
   Function :   opt_med3()
   In       :   pointer to array of 3 pixel values
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 3 pixel values
   Notice   :   found on sci.image.processing
                cannot go faster unless assumptions are made
                on the nature of the input signal.
 ---------------------------------------------------------------------------*/

float opt_med3(float * p)
{
    PIX_SORT(p[0],p[1]) ; PIX_SORT(p[1],p[2]) ; PIX_SORT(p[0],p[1]) ;
    return(p[1]) ;
}

/*----------------------------------------------------------------------------
   Function :   opt_med5()
   In       :   pointer to array of 5 pixel values
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 5 pixel values
   Notice   :   found on sci.image.processing
                cannot go faster unless assumptions are made
                on the nature of the input signal.
 ---------------------------------------------------------------------------*/

float opt_med5(float * p)
{
    PIX_SORT(p[0],p[1]) ; PIX_SORT(p[3],p[4]) ; PIX_SORT(p[0],p[3]) ;
    PIX_SORT(p[1],p[4]) ; PIX_SORT(p[1],p[2]) ; PIX_SORT(p[2],p[3]) ;
    PIX_SORT(p[1],p[2]) ; return(p[2]) ;
}

/*----------------------------------------------------------------------------
   Function :   opt_med6()
   In       :   pointer to array of 6 pixel values
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 6 pixel values
   Notice   :   from Christoph_John@gmx.de
                based on a selection network which was proposed in
                "FAST, EFFICIENT MEDIAN FILTERS WITH EVEN LENGTH WINDOWS"
                J.P. HAVLICEK, K.A. SAKADY, G.R.KATZ
                If you need larger even length kernels check the paper
 ---------------------------------------------------------------------------*/

float opt_med6(float * p)
{
    PIX_SORT(p[1], p[2]); PIX_SORT(p[3],p[4]);
    PIX_SORT(p[0], p[1]); PIX_SORT(p[2],p[3]); PIX_SORT(p[4],p[5]);
    PIX_SORT(p[1], p[2]); PIX_SORT(p[3],p[4]);
    PIX_SORT(p[0], p[1]); PIX_SORT(p[2],p[3]); PIX_SORT(p[4],p[5]);
    PIX_SORT(p[1], p[2]); PIX_SORT(p[3],p[4]);
    return ( p[2] + p[3] ) * 0.5;
    /* PIX_SORT(p[2], p[3]) results in lower median in p[2] and upper median in p[3] */
}


/*----------------------------------------------------------------------------
   Function :   opt_med7()
   In       :   pointer to array of 7 pixel values
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 7 pixel values
   Notice   :   found on sci.image.processing
                cannot go faster unless assumptions are made
                on the nature of the input signal.
 ---------------------------------------------------------------------------*/

float opt_med7(float * p)
{
    PIX_SORT(p[0], p[5]) ; PIX_SORT(p[0], p[3]) ; PIX_SORT(p[1], p[6]) ;
    PIX_SORT(p[2], p[4]) ; PIX_SORT(p[0], p[1]) ; PIX_SORT(p[3], p[5]) ;
    PIX_SORT(p[2], p[6]) ; PIX_SORT(p[2], p[3]) ; PIX_SORT(p[3], p[6]) ;
    PIX_SORT(p[4], p[5]) ; PIX_SORT(p[1], p[4]) ; PIX_SORT(p[1], p[3]) ;
    PIX_SORT(p[3], p[4]) ; return (p[3]) ;
}

/*----------------------------------------------------------------------------
   Function :   opt_med9()
   In       :   pointer to an array of 9 pixelvalues
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 9 pixelvalues
   Notice   :   in theory, cannot go faster without assumptions on the
                signal.
                Formula from:
                XILINX XCELL magazine, vol. 23 by John L. Smith
  
                The input array is modified in the process
                The result array is guaranteed to contain the median
                value
                in middle position, but other elements are NOT sorted.
 ---------------------------------------------------------------------------*/

float opt_med9(float * p)
{
    PIX_SORT(p[1], p[2]) ; PIX_SORT(p[4], p[5]) ; PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[1]) ; PIX_SORT(p[3], p[4]) ; PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[1], p[2]) ; PIX_SORT(p[4], p[5]) ; PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[3]) ; PIX_SORT(p[5], p[8]) ; PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[3], p[6]) ; PIX_SORT(p[1], p[4]) ; PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[4], p[7]) ; PIX_SORT(p[4], p[2]) ; PIX_SORT(p[6], p[4]) ;
    PIX_SORT(p[4], p[2]) ; return(p[4]) ;
}


/*----------------------------------------------------------------------------
   Function :   opt_med25()
   In       :   pointer to an array of 25 pixelvalues
   Out      :   a pixelvalue
   Job      :   optimized search of the median of 25 pixelvalues
   Notice   :   in theory, cannot go faster without assumptions on the
                signal.
  				Code taken from Graphic Gems.
 ---------------------------------------------------------------------------*/

float opt_med25(float * p)
{


    PIX_SORT(p[0], p[1]) ;   PIX_SORT(p[3], p[4]) ;   PIX_SORT(p[2], p[4]) ;
    PIX_SORT(p[2], p[3]) ;   PIX_SORT(p[6], p[7]) ;   PIX_SORT(p[5], p[7]) ;
    PIX_SORT(p[5], p[6]) ;   PIX_SORT(p[9], p[10]) ;  PIX_SORT(p[8], p[10]) ;
    PIX_SORT(p[8], p[9]) ;   PIX_SORT(p[12], p[13]) ; PIX_SORT(p[11], p[13]) ;
    PIX_SORT(p[11], p[12]) ; PIX_SORT(p[15], p[16]) ; PIX_SORT(p[14], p[16]) ;
    PIX_SORT(p[14], p[15]) ; PIX_SORT(p[18], p[19]) ; PIX_SORT(p[17], p[19]) ;
    PIX_SORT(p[17], p[18]) ; PIX_SORT(p[21], p[22]) ; PIX_SORT(p[20], p[22]) ;
    PIX_SORT(p[20], p[21]) ; PIX_SORT(p[23], p[24]) ; PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[3], p[6]) ;   PIX_SORT(p[0], p[6]) ;   PIX_SORT(p[0], p[3]) ;
    PIX_SORT(p[4], p[7]) ;   PIX_SORT(p[1], p[7]) ;   PIX_SORT(p[1], p[4]) ;
    PIX_SORT(p[11], p[14]) ; PIX_SORT(p[8], p[14]) ;  PIX_SORT(p[8], p[11]) ;
    PIX_SORT(p[12], p[15]) ; PIX_SORT(p[9], p[15]) ;  PIX_SORT(p[9], p[12]) ;
    PIX_SORT(p[13], p[16]) ; PIX_SORT(p[10], p[16]) ; PIX_SORT(p[10], p[13]) ;
    PIX_SORT(p[20], p[23]) ; PIX_SORT(p[17], p[23]) ; PIX_SORT(p[17], p[20]) ;
    PIX_SORT(p[21], p[24]) ; PIX_SORT(p[18], p[24]) ; PIX_SORT(p[18], p[21]) ;
    PIX_SORT(p[19], p[22]) ; PIX_SORT(p[8], p[17]) ;  PIX_SORT(p[9], p[18]) ;
    PIX_SORT(p[0], p[18]) ;  PIX_SORT(p[0], p[9]) ;   PIX_SORT(p[10], p[19]) ;
    PIX_SORT(p[1], p[19]) ;  PIX_SORT(p[1], p[10]) ;  PIX_SORT(p[11], p[20]) ;
    PIX_SORT(p[2], p[20]) ;  PIX_SORT(p[2], p[11]) ;  PIX_SORT(p[12], p[21]) ;
    PIX_SORT(p[3], p[21]) ;  PIX_SORT(p[3], p[12]) ;  PIX_SORT(p[13], p[22]) ;
    PIX_SORT(p[4], p[22]) ;  PIX_SORT(p[4], p[13]) ;  PIX_SORT(p[14], p[23]) ;
    PIX_SORT(p[5], p[23]) ;  PIX_SORT(p[5], p[14]) ;  PIX_SORT(p[15], p[24]) ;
    PIX_SORT(p[6], p[24]) ;  PIX_SORT(p[6], p[15]) ;  PIX_SORT(p[7], p[16]) ;
    PIX_SORT(p[7], p[19]) ;  PIX_SORT(p[13], p[21]) ; PIX_SORT(p[15], p[23]) ;
    PIX_SORT(p[7], p[13]) ;  PIX_SORT(p[7], p[15]) ;  PIX_SORT(p[1], p[9]) ;
    PIX_SORT(p[3], p[11]) ;  PIX_SORT(p[5], p[17]) ;  PIX_SORT(p[11], p[17]) ;
    PIX_SORT(p[9], p[17]) ;  PIX_SORT(p[4], p[10]) ;  PIX_SORT(p[6], p[12]) ;
    PIX_SORT(p[7], p[14]) ;  PIX_SORT(p[4], p[6]) ;   PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[12], p[14]) ; PIX_SORT(p[10], p[14]) ; PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[10], p[12]) ; PIX_SORT(p[6], p[10]) ;  PIX_SORT(p[6], p[17]) ;
    PIX_SORT(p[12], p[17]) ; PIX_SORT(p[7], p[17]) ;  PIX_SORT(p[7], p[10]) ;
    PIX_SORT(p[12], p[18]) ; PIX_SORT(p[7], p[12]) ;  PIX_SORT(p[10], p[18]) ;
    PIX_SORT(p[12], p[20]) ; PIX_SORT(p[10], p[20]) ; PIX_SORT(p[10], p[12]) ;

    return (p[12]);
}

/*
 * For arbitrary filter window Size use this method from
 * https://embeddedgurus.com/stack-overflow/2010/10/median-filtering/
 */
#define 	STOPPER_FLOATS -3.4e38                                      /* Smaller than any datum */
#define    	MEDIAN_ARBITRARY_FILTER_SIZE_FLOATS    (11)

float medianFilterArbitraryFloats(float datum)
{
 struct pair
 {
   struct pair   *point;                              /* Pointers forming list linked in sorted order */
   float  value;                                   /* Values to sort */
 };
 static struct pair buffer[MEDIAN_ARBITRARY_FILTER_SIZE_FLOATS] = {0}; /* Buffer of nwidth pairs */
 static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
 static struct pair small = {NULL, STOPPER_FLOATS};          /* Chain stopper */
 static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

 struct pair *successor;                              /* Pointer to successor of replaced data item */
 struct pair *scan;                                   /* Pointer used to scan down the sorted list */
 struct pair *scanold;                                /* Previous value of scan */
 struct pair *median;                                 /* Pointer to median */
 uint16_t i;

 if (datum == STOPPER_FLOATS)
 {
   datum = STOPPER_FLOATS + 1;                             /* No stoppers allowed. */
 }

 if ( (++datpoint - buffer) >= MEDIAN_ARBITRARY_FILTER_SIZE_FLOATS)
 {
   datpoint = buffer;                               /* Increment and wrap data in pointer.*/
 }

 datpoint->value = datum;                           /* Copy in new datum */
 successor = datpoint->point;                       /* Save pointer to old value's successor */
 median = &big;                                     /* Median initially to first in chain */
 scanold = NULL;                                    /* Scanold initially null. */
 scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

 /* Handle chain-out of first item in chain as special case */
 if (scan->point == datpoint)
 {
   scan->point = successor;
 }
 scanold = scan;                                     /* Save this pointer and   */
 scan = scan->point ;                                /* step down chain */

 /* Loop through the chain, normal loop exit via break. */
 for (i = 0 ; i < MEDIAN_ARBITRARY_FILTER_SIZE_FLOATS; ++i)
 {
   /* Handle odd-numbered item in chain  */
   if (scan->point == datpoint)
   {
     scan->point = successor;                      /* Chain out the old datum.*/
   }

   if (scan->value < datum)                        /* If datum is larger than scanned value,*/
   {
     datpoint->point = scanold->point;             /* Chain it in here.  */
     scanold->point = datpoint;                    /* Mark it chained in. */
     datum = STOPPER_FLOATS;
   };

   /* Step median pointer down chain after doing odd-numbered element */
   median = median->point;                       /* Step median pointer.  */
   if (scan == &small)
   {
     break;                                      /* Break at end of chain  */
   }
   scanold = scan;                               /* Save this pointer and   */
   scan = scan->point;                           /* step down chain */

   /* Handle even-numbered item in chain.  */
   if (scan->point == datpoint)
   {
     scan->point = successor;
   }

   if (scan->value < datum)
   {
     datpoint->point = scanold->point;
     scanold->point = datpoint;
     datum = STOPPER_FLOATS;
   }

   if (scan == &small)
   {
     break;
   }

   scanold = scan;
   scan = scan->point;
 }
 return median->value;
}

/*
 * Same function as above, but for uint16_t values
 */

#define 	STOPPER_UINT16 0                                      /* Smaller than any datum */
#define    	MEDIAN_ARBITRARY_FILTER_SIZE_UINT16    (11)

uint16_t medianFilterArbitraryUint16(uint16_t datum)
{
 struct pair
 {
   struct pair   *point;                              /* Pointers forming list linked in sorted order */
   uint16_t  value;                                   /* Values to sort */
 };
 static struct pair buffer[MEDIAN_ARBITRARY_FILTER_SIZE_UINT16] = {0}; /* Buffer of nwidth pairs */
 static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
 static struct pair small = {NULL, STOPPER_UINT16};          /* Chain stopper */
 static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

 struct pair *successor;                              /* Pointer to successor of replaced data item */
 struct pair *scan;                                   /* Pointer used to scan down the sorted list */
 struct pair *scanold;                                /* Previous value of scan */
 struct pair *median;                                 /* Pointer to median */
 uint16_t i;

 if (datum == STOPPER_UINT16)
 {
   datum = STOPPER_UINT16 + 1;                             /* No stoppers allowed. */
 }

 if ( (++datpoint - buffer) >= MEDIAN_ARBITRARY_FILTER_SIZE_UINT16)
 {
   datpoint = buffer;                               /* Increment and wrap data in pointer.*/
 }

 datpoint->value = datum;                           /* Copy in new datum */
 successor = datpoint->point;                       /* Save pointer to old value's successor */
 median = &big;                                     /* Median initially to first in chain */
 scanold = NULL;                                    /* Scanold initially null. */
 scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

 /* Handle chain-out of first item in chain as special case */
 if (scan->point == datpoint)
 {
   scan->point = successor;
 }
 scanold = scan;                                     /* Save this pointer and   */
 scan = scan->point ;                                /* step down chain */

 /* Loop through the chain, normal loop exit via break. */
 for (i = 0 ; i < MEDIAN_ARBITRARY_FILTER_SIZE_UINT16; ++i)
 {
   /* Handle odd-numbered item in chain  */
   if (scan->point == datpoint)
   {
     scan->point = successor;                      /* Chain out the old datum.*/
   }

   if (scan->value < datum)                        /* If datum is larger than scanned value,*/
   {
     datpoint->point = scanold->point;             /* Chain it in here.  */
     scanold->point = datpoint;                    /* Mark it chained in. */
     datum = STOPPER_UINT16;
   };

   /* Step median pointer down chain after doing odd-numbered element */
   median = median->point;                       /* Step median pointer.  */
   if (scan == &small)
   {
     break;                                      /* Break at end of chain  */
   }
   scanold = scan;                               /* Save this pointer and   */
   scan = scan->point;                           /* step down chain */

   /* Handle even-numbered item in chain.  */
   if (scan->point == datpoint)
   {
     scan->point = successor;
   }

   if (scan->value < datum)
   {
     datpoint->point = scanold->point;
     scanold->point = datpoint;
     datum = STOPPER_UINT16;
   }

   if (scan == &small)
   {
     break;
   }

   scanold = scan;
   scan = scan->point;
 }
 return median->value;
}


#undef PIX_SORT
#undef PIX_SWAP


