/* 	Battery Temperature

	For calculating the battery temperature we could use the fpu emu or even do it completely with integer math or do a lookup table.
	The result for all 3 options would be exactly the same, all with pros and cons regarding code size or table size. Using a simple
	lookup table is for the moment the best sollution as its fastest and smallest of the 3 options.

	For completeness on how to calculate the battery temperature from scrach here is the used formula.
	formula:
	// x is the raw info we get from smem.
	int temp = ( x * 18 ) / ( 2600 - x );
	int battery_temperature = (int)( ( 1.0 / ( log( double( temp ) * 0.01 ) / 4360.0 ) + ( 1.0 / ( 273.15 + 25.0 ) ) ) - 273.15 ) * 10.0;

	int64 formula:

	#include <math64.h>

	s64 temp_x = ((s64)log_table[ temp ] ) * 1000LL;
	temp_x = (  div_s64( temp_x, 4360 ) ) + 3354016LL;
	temp_x = div64_u64( 100000000000ULL, temp_x );
	temp_x = temp_x - 27315;
	buffer->batt_temp = ( ( int ) temp_x ) / 10;

	int64 table:
	// this is the hacked formula only using integer math
	log_table table is created by using the following code to dump a table
	long long temp = ( i * 18 ) / ( 2600 - i );
	int d_log = (int) log( double( temp * 0.01 ) ) * 1000000.0;

	todo: if you look at the end of the table you see a lot of repeatating numbers, this is because of the integer math in the first
			equalation. We could decrease the table size by adding some hardcoded numbers.
*/

/*	Table created with the following program
	int GetRealPartTemp( int x ) {
		double temp_f = log( double( x ) * 0.01 );
		temp_f = temp_f * ( 1.0 / 4360.0 );
		temp_f = temp_f + ( 1.0 / ( 273.15 + 25.0 ) );
		temp_f = 1.0 / temp_f;
		temp_f = temp_f - 273.15;
		return ( int )( double )( temp_f * 10.0 );
	}

	int main() {
		int table_size = 0, offset = 0, i = 0, temp = 0;
		for ( ; i < 2599; i++ ) {
			temp = GetRealPartTemp(i);

			if ( temp > 900 || temp < -200 ) continue;			// limit for not hotter than 90 degrease not colder than -20
			table_size++;
			if ( offset == 0 ) offset = i;								// save which offset we start.

			fprintf( fp, "%4d, ", temp );

			if ( !(table_size % 10) ) fprintf(fp, "\n\t");			// some formatting
		}
		fprintf(fp, "\n };");
		fprintf(fp, "// offset = %d\n", offset);
		fclose(fp);
		return 0;
	}
*/

/* Charge graph :
 *
 *      Li-ion battery charging is made in two steps :
 *      1 / Constant current charge : During this step, the voltage of the battery reach it's max value.
 *          At the end of this step, the battery charge level is about 70%
 *      2 / Constant voltage charge : During this step, the charge current will decrease until it reach
 *          3% of the rated battery capacity.
 *          The battery can be considered fully charged when the charge current is 3% or less.
 *
 *      1st part of charging process :
 *      ------------------------------
 *                     Voltage (%)
 *                          ^
 *                     100% |                                                        x
 *                          |
 *  max_volt_min_percentage |                                           x
 *                          |
 *  mid_volt_min_percentage |                                x
 *                          |
 *  low_volt_min_percentage |                    x
 *                          |
 *                       0% +--------x-------------------------------------------------> Voltage
 *                                   |           |           |           |           |
 *                            critical_volt   low_volt    mid_volt    max_volt   full_volt
 *                             _threshold   _threshold  _threshold  _threshold  _threshold
 *
 *	2nd part of charging process (C = battery capacity in current -> 900mAh batt => 900mA):
 *	------------------------------
 *                       Current
 *                          ^
 *                        C | x	x
 *                          |
 *                          |        x
 *                          |
 *                          |             x
 *                          |
 *                          |                      x
 *                     3% C |                                x
 *                          +------------------------------------> Percent of charge
 *                               |        |        |         |
 *                              70%                         100%
 *
 * added:
 * the discharging curve can be define via *6 points* and every point can be positioning on the courve (like if you would try to draw the curve via lines
 * therefore you can define a slope value and start height
 * see paramter below, and also you can find on XDA developers a XLS files where you can see the algo how it works in detail
 * also in the XLS can the values be tested (you only need log data from your model - via grep kernellog for "battlog")
*/

/* Percentage calculus :
 *
 *  - portion range in Volts is ( #next portion#_threshold - #current portion#_threshold )
 *  - portion range in % is ( #next portion#_min_percentage - #current portion#_min_percentage )
 *
 *  - % of charge in the current portion of the graph = ( current voltage - #current portion#_threshold) / (portion range in Volts)
 *		This value will be between 0 and 1
 *  - % of charge in the global percentage = % of charge in the current portion of the graph *  (#current portion#_perc_range)
 *		This value will be between 0 and (#current portion#_perc_range)
 *  *****************************************************************************************
 *  * % of charge = % of charge in the global percentage + #current portion#_min_percentage *
 *  *****************************************************************************************
 */

/* Temperature lookup table
 * Tested on RAPH300
 */
static const short temp_table[] = {
	 872,  837,  807,  780,  755,  733,  713,  694,  677,  661,
	 646,  631,  618,  606,  594,  583,  572,  562,  552,  543,
	 534,  525,  517,  509,  501,  494,  487,  480,  473,  467,
	 461,  455,  449,  443,  438,  432,  427,  422,  417,  412,
	 407,  402,  398,  393,  389,  385,  381,  377,  373,  369,
	 365,  361,  357,  354,  350,  347,  343,  340,  337,  333,
	 330,  327,  324,  321,  318,  315,  312,  309,  307,  304,
	 301,  298,  296,  293,  291,  288,  285,  283,  281,  278,
	 276,  273,  271,  269,  267,  264,  262,  260,  258,  256,
	 254,  252,  250,  247,  245,  243,  242,  240,  238,  236,
	 234,  232,  230,  228,  227,  225,  223,  221,  220,  218,
	 216,  214,  213,  211,  210,  208,  206,  205,  203,  202,
	 200,  198,  197,  195,  194,  192,  191,  190,  188,  187,
	 185,  184,  182,  181,  180,  178,  177,  176,  174,  173,
	 172,  170,  169,  168,  167,  165,  164,  163,  162,  160,
	 159,  158,  157,  155,  154,  153,  152,  151,  150,  148,
	 147,  146,  145,  144,  143,  142,  141,  140,  139,  137,
	 136,  135,  134,  133,  132,  131,  130,  129,  128,  127,
	 126,  125,  124,  123,  122,  121,  120,  119,  118,  117,
	 116,  116,  115,  114,  113,  112,  111,  110,  109,  108,
	 107,  106,  106,  105,  104,  103,  102,  101,  100,   99,
	  99,   98,   97,   96,   95,   94,   94,   93,   92,   91,
	  90,   90,   89,   88,   87,   86,   86,   85,   84,   83,
	  83,   82,   81,   80,   80,   79,   78,   77,   77,   76,
	  75,   74,   74,   73,   72,   72,   71,   70,   69,   69,
	  68,   67,   67,   66,   65,   65,   64,   63,   63,   62,
	  61,   61,   60,   59,   59,   58,   57,   57,   56,   55,
	  55,   54,   53,   53,   52,   51,   51,   50,   50,   49,
	  48,   48,   47,   47,   46,   45,   45,   44,   44,   43,
	  42,   42,   41,   41,   40,   39,   39,   38,   38,   37,
	  37,   36,   35,   35,   34,   34,   33,   33,   32,   31,
	  31,   30,   30,   29,   29,   28,   28,   27,   27,   26,
	  26,   25,   24,   24,   23,   23,   22,   22,   21,   21,
	  20,   20,   19,   19,   18,   18,   17,   17,   16,   16,
	  15,   15,   14,   14,   13,   13,   12,   12,   11,   11,
	  10,   10,    9,    9,    8,    8,    7,    7,    7,    6,
	   6,    5,    5,    4,    4,    3,    3,    2,    2,    1,
	   1,    1,    0,    0,    0,    0,   -1,   -1,   -2,   -2,
	  -2,   -3,   -3,   -4,   -4,   -5,   -5,   -6,   -6,   -6,
	  -7,   -7,   -8,   -8,   -9,   -9,   -9,  -10,  -10,  -11,
	 -11,  -11,  -12,  -12,  -13,  -13,  -14,  -14,  -14,  -15,
	 -15,  -16,  -16,  -16,  -17,  -17,  -18,  -18,  -18,  -19,
	 -19,  -20,  -20,  -20,  -21,  -21,  -21,  -22,  -22,  -23,
	 -23,  -23,  -24,  -24,  -25,  -25,  -25,  -26,  -26,  -26,
	 -27,  -27,  -28,  -28,  -28,  -29,  -29,  -29,  -30,  -30,
	 -31,  -31,  -31,  -32,  -32,  -32,  -33,  -33,  -33,  -34,
	 -34,  -34,  -35,  -35,  -36,  -36,  -36,  -37,  -37,  -37,
	 -38,  -38,  -38,  -39,  -39,  -39,  -40,  -40,  -40,  -41,
	 -41,  -41,  -42,  -42,  -42,  -43,  -43,  -43,  -44,  -44,
	 -44,  -45,  -45,  -45,  -46,  -46,  -46,  -47,  -47,  -47,
	 -48,  -48,  -48,  -49,  -49,  -49,  -50,  -50,  -50,  -51,
	 -51,  -51,  -52,  -52,  -52,  -53,  -53,  -53,  -53,  -54,
	 -54,  -54,  -55,  -55,  -55,  -56,  -56,  -56,  -57,  -57,
	 -57,  -57,  -58,  -58,  -58,  -59,  -59,  -59,  -60,  -60,
	 -60,  -60,  -61,  -61,  -61,  -62,  -62,  -62,  -63,  -63,
	 -63,  -63,  -64,  -64,  -64,  -65,  -65,  -65,  -65,  -66,
	 -66,  -66,  -67,  -67,  -67,  -67,  -68,  -68,  -68,  -69,
	 -69,  -69,  -69,  -70,  -70,  -70,  -71,  -71,  -71,  -71,
	 -72,  -72,  -72,  -72,  -73,  -73,  -73,  -74,  -74,  -74,
	 -74,  -75,  -75,  -75,  -75,  -76,  -76,  -76,  -77,  -77,
	 -77,  -77,  -78,  -78,  -78,  -78,  -79,  -79,  -79,  -79,
	 -80,  -80,  -80,  -80,  -81,  -81,  -81,  -82,  -82,  -82,
	 -82,  -83,  -83,  -83,  -83,  -84,  -84,  -84,  -84,  -85,
	 -85,  -85,  -85,  -86,  -86,  -86,  -86,  -87,  -87,  -87,
	 -87,  -88,  -88,  -88,  -88,  -89,  -89,  -89,  -89,  -90,
	 -90,  -90,  -90,  -91,  -91,  -91,  -91,  -91,  -92,  -92,
	 -92,  -92,  -93,  -93,  -93,  -93,  -94,  -94,  -94,  -94,
	 -95,  -95,  -95,  -95,  -96,  -96,  -96,  -96,  -96,  -97,
	 -97,  -97,  -97,  -98,  -98,  -98,  -98,  -99,  -99,  -99,
	 -99,  -99, -100, -100, -100, -100, -101, -101, -101, -101,
	-101, -102, -102, -102, -102, -103, -103, -103, -103, -103,
	-104, -104, -104, -104, -105, -105, -105, -105, -105, -106,
	-106, -106, -106, -107, -107, -107, -107, -107, -108, -108,
	-108, -108, -108, -109, -109, -109, -109, -110, -110, -110,
	-110, -110, -111, -111, -111, -111, -111, -112, -112, -112,
	-112, -112, -113, -113, -113, -113, -113, -114, -114, -114,
	-114, -114, -115, -115, -115, -115, -116, -116, -116, -116,
	-116, -117, -117, -117, -117, -117, -118, -118, -118, -118,
	-118, -119, -119, -119, -119, -119, -120, -120, -120, -120,
	-120, -120, -121, -121, -121, -121, -121, -122, -122, -122,
	-122, -122, -123, -123, -123, -123, -123, -124, -124, -124,
	-124, -124, -125, -125, -125, -125, -125, -125, -126, -126,
	-126, -126, -126, -127, -127, -127, -127, -127, -128, -128,
	-128, -128, -128, -128, -129, -129, -129, -129, -129, -130,
	-130, -130, -130, -130, -130, -131, -131, -131, -131, -131,
	-132, -132, -132, -132, -132, -132, -133, -133, -133, -133,
	-133, -134, -134, -134, -134, -134, -134, -135, -135, -135,
	-135, -135, -135, -136, -136, -136, -136, -136, -137, -137,
	-137, -137, -137, -137, -138, -138, -138, -138, -138, -138,
	-139, -139, -139, -139, -139, -139, -140, -140, -140, -140,
	-140, -140, -141, -141, -141, -141, -141, -142, -142, -142,
	-142, -142, -142, -143, -143, -143, -143, -143, -143, -144,
	-144, -144, -144, -144, -144, -144, -145, -145, -145, -145,
	-145, -145, -146, -146, -146, -146, -146, -146, -147, -147,
	-147, -147, -147, -147, -148, -148, -148, -148, -148, -148,
	-149, -149, -149, -149, -149, -149, -150, -150, -150, -150,
	-150, -150, -150, -151, -151, -151, -151, -151, -151, -152,
	-152, -152, -152, -152, -152, -152, -153, -153, -153, -153,
	-153, -153, -154, -154, -154, -154, -154, -154, -154, -155,
	-155, -155, -155, -155, -155, -156, -156, -156, -156, -156,
	-156, -156, -157, -157, -157, -157, -157, -157, -158, -158,
	-158, -158, -158, -158, -158, -159, -159, -159, -159, -159,
	-159, -159, -160, -160, -160, -160, -160, -160, -160, -161,
	-161, -161, -161, -161, -161, -161, -162, -162, -162, -162,
	-162, -162, -162, -163, -163, -163, -163, -163, -163, -164,
	-164, -164, -164, -164, -164, -164, -165, -165, -165, -165,
	-165, -165, -165, -165, -166, -166, -166, -166, -166, -166,
	-166, -167, -167, -167, -167, -167, -167, -167, -168, -168,
	-168, -168, -168, -168, -168, -169, -169, -169, -169, -169,
	-169, -169, -170, -170, -170, -170, -170, -170, -170, -170,
	-171, -171, -171, -171, -171, -171, -171, -172, -172, -172,
	-172, -172, -172, -172, -172, -173, -173, -173, -173, -173,
	-173, -173, -174, -174, -174, -174, -174, -174, -174, -174,
	-175, -175, -175, -175, -175, -175, -175, -176, -176, -176,
	-176, -176, -176, -176, -176, -177, -177, -177, -177, -177,
	-177, -177, -177, -178, -178, -178, -178, -178, -178, -178,
	-179, -179, -179, -179, -179, -179, -179, -179, -180, -180,
	-180, -180, -180, -180, -180, -180, -181, -181, -181, -181,
	-181, -181, -181, -181, -182, -182, -182, -182, -182, -182,
	-182, -182, -183, -183, -183, -183, -183, -183, -183, -183,
	-184, -184, -184, -184, -184, -184, -184, -184, -185, -185,
	-185, -185, -185, -185, -185, -185, -185, -186, -186, -186,
	-186, -186, -186, -186, -186, -187, -187, -187, -187, -187,
	-187, -187, -187, -188, -188, -188, -188, -188, -188, -188,
	-188, -188, -189, -189, -189, -189, -189, -189, -189, -189,
	-190, -190, -190, -190, -190, -190, -190, -190, -190, -191,
	-191, -191, -191, -191, -191, -191, -191, -192, -192, -192,
	-192, -192, -192, -192, -192, -192, -193, -193, -193, -193,
	-193, -193, -193, -193, -194, -194, -194, -194, -194, -194,
	-194, -194, -194, -195, -195, -195, -195, -195, -195, -195,
	-195, -195, -196, -196, -196, -196, -196, -196, -196, -196,
	-196, -197, -197, -197, -197, -197, -197, -197, -197, -197,
	-198, -198, -198, -198, -198, -198, -198, -198, -198, -199,
	-199, -199, -199, -199, -199, -199, -199, -199, -200, -200,
	-200, -200, -200, -200, -200, -200, -200
 };

struct sBattery_Parameters
{
	/* Charge graph :
	 * 	See htc_battery_smem.h for details
	 * see Battery_Calculator.xls -> can be found on XDA-Developers
	 */                                                            
	int battery_capacity;               /* Battery capacity (mAh) */
	int termination_current;            /* Charge termination current (typically 3% of battery_capacity) */
	int temp_correction;
	int temp_correction_const;
	int volt_discharge_res_coeff;

        //definition of points
	int cri_volt_threshold;        	    /* Point 1 - Lowest voltage value the critical voltage part of the graph */
	int low_volt_threshold;             /* Point 2 Lowest voltage value the low voltage part of the graph */
	int min_volt_threshold;             /* Point 3 Mininimum voltage value the mid voltage part of the graph */
	int mid_volt_threshold;             /* Point 4 Lowest voltage value the mid voltage part of the graph */
	int med_volt_threshold;             /* Point 5 Lowest voltage value the mid voltage part of the graph */
	int max_volt_threshold;             /* Point 6 Lowest voltage value the max voltage part of the graph */
	int full_volt_threshold;     /* only for define when battery is on 99% or higher almost full voltage of the battery */
	// percent cure start min percentage (% x 10)
	int cri_volt_perc_start; 
	int low_volt_perc_start;
	int min_volt_perc_start;
	int mid_volt_perc_start;
	int med_volt_perc_start;
	int max_volt_perc_start;
	//dynamic slope (centi-unti) (unit x 10)
	int cri_volt_dynslope;
	int low_volt_dynslope;
	int min_volt_dynslope;
	int mid_volt_dynslope;
	int med_volt_dynslope;
	int max_volt_dynslope;
} ;


/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz
 * Batt Vendor 1
 */
static const struct sBattery_Parameters sBatParams_Topaz_1100mAh_v1 =
{
    .battery_capacity =			1100,
    .termination_current =		70,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4110,
    .cri_volt_perc_start =		-10,
    .low_volt_perc_start =		75,
    .min_volt_perc_start =		300,
    .mid_volt_perc_start =		420,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		930,
    .cri_volt_dynslope =		220,
    .low_volt_dynslope =		44,
    .min_volt_dynslope =		28,
    .mid_volt_dynslope =		43,
    .med_volt_dynslope =		68,
    .max_volt_dynslope =		96,
};

/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz
 * Batt Vendor 2 and >5
 */
static const struct sBattery_Parameters sBatParams_Topaz_1100mAh_v2 =
{
    .battery_capacity =			1100,
    .termination_current =		70,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3340,
    .low_volt_threshold =		3615,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3750,
    .med_volt_threshold =		3820,
    .max_volt_threshold =		3980,
    .full_volt_threshold =		4160,
    .cri_volt_perc_start =		0,
    .low_volt_perc_start =		45,
    .min_volt_perc_start =		220,
    .mid_volt_perc_start =		405,
    .med_volt_perc_start =		585,
    .max_volt_perc_start =		805,
    .cri_volt_dynslope =		600,
    .low_volt_dynslope =		48,
    .min_volt_dynslope =		27,
    .mid_volt_dynslope =		39,
    .med_volt_dynslope =		73,
    .max_volt_dynslope =		95,
};


/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz
 * Batt Vendor 3
 */
static const struct sBattery_Parameters sBatParams_Topaz_1350mAh_v3 =
{
    .battery_capacity =			1350,
    .termination_current =		33,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4110,
    .cri_volt_perc_start =		-10,
    .low_volt_perc_start =		75,
    .min_volt_perc_start =		300,
    .mid_volt_perc_start =		420,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		930,
    .cri_volt_dynslope =		220,
    .low_volt_dynslope =		44,
    .min_volt_dynslope =		28,
    .mid_volt_dynslope =		43,
    .med_volt_dynslope =		68,
    .max_volt_dynslope =		96,
};

/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz
 * Batt Vendor 4
 */
static const struct sBattery_Parameters sBatParams_Topaz_1350mAh_v4 =
{
    .battery_capacity =			1350,
    .termination_current =		33,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4110,
    .cri_volt_perc_start =		-10,
    .low_volt_perc_start =		75,
    .min_volt_perc_start =		300,
    .mid_volt_perc_start =		420,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		930,
    .cri_volt_dynslope =		220,
    .low_volt_dynslope =		44,
    .min_volt_dynslope =		28,
    .mid_volt_dynslope =		43,
    .med_volt_dynslope =		68,
    .max_volt_dynslope =		96,
};

/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz
 * Batt Vendor 5
 */
static const struct sBattery_Parameters sBatParams_Topaz_2150mAh =
{
    .battery_capacity =			2150,
    .termination_current =		33,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4110,
    .cri_volt_perc_start =		-10,
    .low_volt_perc_start =		75,
    .min_volt_perc_start =		300,
    .mid_volt_perc_start =		420,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		930,
    .cri_volt_dynslope =		220,
    .low_volt_dynslope =		44,
    .min_volt_dynslope =		28,
    .mid_volt_dynslope =		43,
    .med_volt_dynslope =		68,
    .max_volt_dynslope =		96,
};

/* Extracted from battdrvr.dll ver.5.8.0.0 of HTC Raphael */
static const struct sBattery_Parameters sBatParams_Raphael_1800mAh =
{
    .battery_capacity =			2150,
    .termination_current =		33,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4110,
    .cri_volt_perc_start =		-10,
    .low_volt_perc_start =		75,
    .min_volt_perc_start =		300,
    .mid_volt_perc_start =		420,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		930,
    .cri_volt_dynslope =		220,
    .low_volt_dynslope =		44,
    .min_volt_dynslope =		28,
    .mid_volt_dynslope =		43,
    .med_volt_dynslope =		68,
    .max_volt_dynslope =		96,
};

//for blackstone original akku pack 1350
static const struct sBattery_Parameters sBatParams_Blackstone_1350mAh =
{
    .battery_capacity =			1350,
    .termination_current =		75,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3300,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3720,
    .mid_volt_threshold =		3755,
    .med_volt_threshold =		3820,
    .max_volt_threshold =		4020,
    .full_volt_threshold =		4150,
    .cri_volt_perc_start =		-40,
    .low_volt_perc_start =		70,
    .min_volt_perc_start =		245,
    .mid_volt_perc_start =		390,
    .med_volt_perc_start =		555,
    .max_volt_perc_start =		860,
    .cri_volt_dynslope =		270,
    .low_volt_dynslope =		70,
    .min_volt_dynslope =		24,
    .mid_volt_dynslope =		39,
    .med_volt_dynslope =		65,
    .max_volt_dynslope =		100,
};

//camel - i add it for blackstone
static const struct sBattery_Parameters sBatParams_Blackstone_1500mAh =
{
    .battery_capacity =			1500,
    .termination_current =		75,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3300,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3720,
    .mid_volt_threshold =		3755,
    .med_volt_threshold =		3820,
    .max_volt_threshold =		4020,
    .full_volt_threshold =		4180,
    .cri_volt_perc_start =		10,
    .low_volt_perc_start =		105,
    .min_volt_perc_start =		260,
    .mid_volt_perc_start =		390,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		860,
    .cri_volt_dynslope =		200,
    .low_volt_dynslope =		65,
    .min_volt_dynslope =		26,
    .mid_volt_dynslope =		38,
    .med_volt_dynslope =		70,
    .max_volt_dynslope =		110,
};

static const struct sBattery_Parameters sBatParams_900mAh =
{
    .battery_capacity =			900,
    .termination_current =		80,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4000,
    .full_volt_threshold =		4140,
    .cri_volt_perc_start =		10,
    .low_volt_perc_start =		105,
    .min_volt_perc_start =		260,
    .mid_volt_perc_start =		390,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		860,
    .cri_volt_dynslope =		200,
    .low_volt_dynslope =		65,
    .min_volt_dynslope =		26,
    .mid_volt_dynslope =		38,
    .med_volt_dynslope =		70,
    .max_volt_dynslope =		110,
};

/* Same values in battdrvr.dll from diamond and raphael */
static const struct sBattery_Parameters sBatParams_1340mAh =
{
    .battery_capacity =			1340,
    .termination_current =		53,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3340,
    .low_volt_threshold =		3615,
    .min_volt_threshold =		3680,
    .mid_volt_threshold =		3750,
    .med_volt_threshold =		3815,
    .max_volt_threshold =		3950,
    .full_volt_threshold =		4155,
    .cri_volt_perc_start =		15,
    .low_volt_perc_start =		65,
    .min_volt_perc_start =		195,
    .mid_volt_perc_start =		405,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		765,
    .cri_volt_dynslope =		600,
    .low_volt_dynslope =		48,
    .min_volt_dynslope =		34,
    .mid_volt_dynslope =		41,
    .med_volt_dynslope =		69,
    .max_volt_dynslope =		83,
};

static const struct sBattery_Parameters sBatParams_1800mAh =
{
    .battery_capacity =			1800,
    .termination_current =		33,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3400,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3700,
    .mid_volt_threshold =		3735,
    .med_volt_threshold =		3800,
    .max_volt_threshold =		4000,
    .full_volt_threshold =		4140,
    .cri_volt_perc_start =		10,
    .low_volt_perc_start =		105,
    .min_volt_perc_start =		260,
    .mid_volt_perc_start =		390,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		860,
    .cri_volt_dynslope =		200,
    .low_volt_dynslope =		65,
    .min_volt_dynslope =		26,
    .mid_volt_dynslope =		38,
    .med_volt_dynslope =		70,
    .max_volt_dynslope =		110,
};

static const struct sBattery_Parameters sBatParams_Kovsky_1500mAh =
{
    .battery_capacity =			1500,
    .termination_current =		85,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3300,
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3720,
    .mid_volt_threshold =		3755,
    .med_volt_threshold =		3820,
    .max_volt_threshold =		4020,
    .full_volt_threshold =		4180,
    .cri_volt_perc_start =		10,
    .low_volt_perc_start =		105,
    .min_volt_perc_start =		260,
    .mid_volt_perc_start =		390,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		860,
    .cri_volt_dynslope =		200,
    .low_volt_dynslope =		65,
    .min_volt_dynslope =		26,
    .mid_volt_dynslope =		38,
    .med_volt_dynslope =		70,
    .max_volt_dynslope =		110,
};

static const struct sBattery_Parameters sBatParams_Kovsky_2000mAh =
{
	.battery_capacity		= 1500,
	.termination_current		= 85,
	.temp_correction		= 0,
	.temp_correction_const		= 16,
	.volt_discharge_res_coeff	= 27,
	.cri_volt_threshold		= 3300,
	.low_volt_threshold		= 3600,
	.min_volt_threshold		= 3720,
	.mid_volt_threshold		= 3755,
	.med_volt_threshold		= 3820,
	.max_volt_threshold		= 4020,
	.full_volt_threshold		= 4180,
	.cri_volt_perc_start		= 10,
	.low_volt_perc_start		= 105,
	.min_volt_perc_start		= 260,
	.mid_volt_perc_start		= 390,
	.med_volt_perc_start		= 570,
	.max_volt_perc_start		= 860,
	.cri_volt_dynslope		= 200,
	.low_volt_dynslope		= 65,
	.min_volt_dynslope		= 26,
	.mid_volt_dynslope		= 38,
	.med_volt_dynslope		= 70,
	.max_volt_dynslope		= 110,
};

static const struct sBattery_Parameters sBatParams_Rhodium300_1500mAh =
{
    .battery_capacity =			1500,
    .termination_current =		58,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3300,
    .low_volt_threshold =		3550,
    .min_volt_threshold =		3660,
    .mid_volt_threshold =		3760,
    .med_volt_threshold =		3810,
    .max_volt_threshold =		3990,
    .full_volt_threshold =		4190,
    .cri_volt_perc_start =		15,
    .low_volt_perc_start =		55,
    .min_volt_perc_start =		140,
    .mid_volt_perc_start =		295,
    .med_volt_perc_start =		515,
    .max_volt_perc_start =		821,
    .cri_volt_dynslope =		650,
    .low_volt_dynslope =		125,
    .min_volt_dynslope =		66,
    .mid_volt_dynslope =		23,
    .med_volt_dynslope =		57,
    .max_volt_dynslope =		115,
};

static const struct sBattery_Parameters sBatParams_Rhodium_1500mAh =
{
    .battery_capacity =			1500,
    .termination_current =		65,
    .temp_correction =			0,
    .temp_correction_const =		16,
    .volt_discharge_res_coeff =		27,
    .cri_volt_threshold =		3220,
    .low_volt_threshold =		3590,
    .min_volt_threshold =		3690,
    .mid_volt_threshold =		3770,
    .med_volt_threshold =		3870,
    .max_volt_threshold =		4050,
    .full_volt_threshold =		4130,
    .cri_volt_perc_start =		0,
    .low_volt_perc_start =		125,
    .min_volt_perc_start =		260,
    .mid_volt_perc_start =		550,
    .med_volt_perc_start =		710,
    .max_volt_perc_start =		925,
    .cri_volt_dynslope =		300,
    .low_volt_dynslope =		73,
    .min_volt_dynslope =		27,
    .mid_volt_dynslope =		63,
    .med_volt_dynslope =		83,
    .max_volt_dynslope =		120,
};

static const struct sBattery_Parameters sBatParams_Rhodium_2200mAh =
{
	.battery_capacity		= 2200,
	.termination_current		= 85,
	.temp_correction		= 0,
	.temp_correction_const		= 16,
	.volt_discharge_res_coeff	= 27,
	.cri_volt_threshold		= 3300,
	.low_volt_threshold		= 3600,
	.min_volt_threshold		= 3720,
	.mid_volt_threshold		= 3755,
	.med_volt_threshold		= 3820,
	.max_volt_threshold		= 4020,
	.full_volt_threshold		= 4180,
	.cri_volt_perc_start		= 10,
	.low_volt_perc_start		= 105,
	.min_volt_perc_start		= 260,
	.mid_volt_perc_start		= 390,
	.med_volt_perc_start		= 570,
	.max_volt_perc_start		= 860,
	.cri_volt_dynslope		= 200,
	.low_volt_dynslope		= 65,
	.min_volt_dynslope		= 26,
	.mid_volt_dynslope		= 38,
	.med_volt_dynslope		= 70,
	.max_volt_dynslope		= 110,
};



/* These values are the orginal from this files. They are present in Raphael battdrvr.dll ver.5.8.0.0 */
static const struct sBattery_Parameters* sBatParams_kovsky[] =
{
	&sBatParams_Kovsky_1500mAh,            /* batt_vendor = 1 */
	&sBatParams_Kovsky_2000mAh,            /* batt_vendor = 2 */
};

static const struct sBattery_Parameters* sBatParams_rhodium[] =
{
	&sBatParams_Rhodium_1500mAh,            /* batt_vendor = 1 */
	&sBatParams_Rhodium_2200mAh,            /* batt_vendor = 2 */
	&sBatParams_Rhodium300_1500mAh,         /* batt_vendor = 1 + VREF=1254 + HWBOARDID=80*/
};

/* These values are the orginal from this files. They are present in Raphael battdrvr.dll ver.5.8.0.0 */
static const struct sBattery_Parameters* sBatParams_diamond[] =
{
	&sBatParams_1340mAh,            /* batt_vendor = 1 */
	&sBatParams_900mAh,             /* batt_vendor = 2 */
};

/* Extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz */
static const struct sBattery_Parameters* sBatParams_topaz[] =
{
	&sBatParams_Topaz_1100mAh_v1,   /* batt_vendor = 1 */
	&sBatParams_Topaz_1100mAh_v2,   /* batt_vendor = 2 */
	&sBatParams_Topaz_1350mAh_v3,   /* batt_vendor = 3 */
	&sBatParams_Topaz_1350mAh_v4,   /* batt_vendor = 4 */
	&sBatParams_Topaz_2150mAh,      /* batt_vendor = 5 */
	/* Batt vendor > 5 are redirected to sBatParams_Topaz_1100mAh_v2 */
};

/* Extracted from battdrvr.dll ver.5.8.0.0 of HTC Raphael */
static const struct sBattery_Parameters* sBatParams_raphael[] =
{
	&sBatParams_Raphael_1800mAh,   /* batt_vendor = 1 */
	&sBatParams_1340mAh,   /* batt_vendor = 2 */
};

//camel - added for blackstone
/* Got from log in WM via kernelog testings  */
static const struct sBattery_Parameters* sBatParams_blackstone[] =
{
	&sBatParams_Blackstone_1500mAh, /* batt_vendor = 1 */
	&sBatParams_Blackstone_1350mAh, /* batt_vendor = 2 default */
};


static const struct sBattery_Parameters* sBatParams_generic[] =
{
	&sBatParams_1800mAh,           /* batt_vendor = 1 */
	&sBatParams_1340mAh,           /* batt_vendor = 2 */
};

