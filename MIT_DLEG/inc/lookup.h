/*
 * error_correction.h
 *
 *  Created on: Jun 21, 2019
 *      Setup by: matt
 *      retrieved from: https://www.embeddedrelated.com/showcode/345.php
 */

#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_LOOKUP_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_LOOKUP_H_


/* Header file */
#if !defined(_LOOKUP_H)
#define _LOOKUP_H

/**
 * @file
 * Table lookup with interpolation (1-D and 2-D) header file.
 * https://www.embeddedrelated.com/showcode/345.php
 */

#include <stdbool.h>

/** One dimensional lookup table. */
typedef const struct
{
  /** Number of elements in the table.  This must be at least 2. */
  unsigned char ncols;
  /** List of input values. */
  float *columns;
  /** Table data (output values).  The output values list must have the same
      length as the input list. */
  float *table;
} Table1d;

/** Two dimensional lookup table. */
typedef const struct
{
  /** Number of columns (X values) in the table.  Must be at least 2. */
  unsigned char ncols;
  /** Number of rows (Y values) in the table.  Must be at least 2. */
  unsigned char nrows;
  /** X-axis input values list. */
  float *columns;
  /** Y-axis input values list. */
  float *rows;
  /** Table data.  This is an array of <code> columns </code> X <code> rows </code>,
      arranged in rows.  For example, <code> table[1] </code> is the second
      column in the first row. */
  float *table;
} Table2d;

/* Prototypes */
bool lookup1d (Table1d *t, float ix, float *o);
bool lookup2d (Table2d *t, float ix, float iy, float *o);


#endif


#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_LOOKUP_H_ */
