The files in this folder contain the same input data used in TestCoordinates.cpp along with some matlab scripts that perform the same calculations found in coordinates.h
The output from matlab is used to validate the coordinates.h calculations in the NUBOTS codebase.
Test input data can be found in both matlab and c++ formats. The matlab format data is passed into the functions starting with calc_****.m to produce the output data.
Test output data provided is in a cpp format.

NOTE: During initial testing when values at the extremes of the data types domain, DBL_MAX and DBL_MIN, were found to cause inf return values when passed to the functions
being tested. While normally some measures would have been put in place to handle this, it was determined that the performance impact would be to great, and that these values won't be
possible during normal operation.
The maximum test values that were determined to be safe are:
TEST_DBL_MAX    = 5.9667e+153;
TEST_DBL_MIN    = 1.4853e-307;

The values returned compared to the pre-calculated values should be within the error threshold:
ERROR_THRESHOLD = 1e-6;
