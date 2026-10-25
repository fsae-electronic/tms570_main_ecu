#include "traction.h"

/* TC (TC1) level -> slip allowed before cutting. [0] unused (TC off). */
static const float s_tc_target_slip[ TC_LEVEL_MAX + 1U ] =
{
    0.0f, 0.40f, 0.34f, 0.29f, 0.24f, 0.20f, 0.17f, 0.14f, 0.11f, 0.08f, 0.06f
};

/* TC Cut (TC2) level -> how hard the cut bites. [0] unused. */
typedef struct
{
    float kp;        /* torque cut per unit of slip error */
    float ki;        /* integral build-up per unit of slip error per second */
    float max_cut;   /* never cut more than this fraction of the request */
    float rise_rate; /* max cut increase per second */
} TcCutLevel_t;

static const TcCutLevel_t s_tc_cut[ TC_CUT_LEVEL_MAX + 1U ] =
{
    { 0.0f,  0.0f,  0.0f,  0.0f },
    { 0.3f,  1.0f,  0.40f, 4.0f },
    { 0.4f,  1.5f,  0.50f, 5.0f },
    { 0.5f,  2.0f,  0.60f, 6.5f },
    { 0.65f, 2.5f,  0.70f, 8.0f },
    { 0.8f,  3.0f,  0.80f, 10.0f }, /* 0 -> 80 % in ~80 ms; launch slip reaches ~1 in ~140 ms */
    { 1.0f,  4.0f,  0.85f, 12.0f },
    { 1.2f,  5.0f,  0.90f, 15.0f },
    { 1.5f,  6.5f,  0.90f, 18.0f },
    { 1.8f,  8.0f,  0.95f, 22.0f },
    { 2.2f,  10.0f, 0.95f, 25.0f }
};

static float s_integral[ 2 ];
static float s_front_f;
static bool  s_front_seeded;

/* Last computed values, for watching in the debugger. */
static float s_slip[ 2 ];
static float s_cut[ 2 ];

static float clamp( float x, float lo, float hi )
{
    if( x < lo )
    {
        return lo;
    }
    if( x > hi )
    {
        return hi;
    }
    return x;
}

/* Move the applied cut towards target without stepping faster than the
 * rise/fall rates allow. */
static float ramp_cut( float cut, float target, float rise_rate, float dt_s )
{
    float max_up = rise_rate * dt_s;
    float max_down = TC_CUT_FALL_RATE * dt_s;

    return cut + clamp( target - cut, -max_down, max_up );
}

static void reset_wheel( uint32_t n )
{
    s_integral[ n ] = 0.0f;
    s_slip[ n ] = 0.0f;
    s_cut[ n ] = 0.0f;
}

/* Lost the slip reference mid-cut: drop the controller state but let the
 * applied cut fade out instead of handing the full request back at once. */
static void release_wheel( uint32_t n, int16_t torque[ 2 ], float dt_s )
{
    s_integral[ n ] = 0.0f;
    s_slip[ n ] = 0.0f;
    s_cut[ n ] = ramp_cut( s_cut[ n ], 0.0f, 0.0f, dt_s );

    torque[ n ] = ( int16_t ) ( ( float ) torque[ n ] * ( 1.0f - s_cut[ n ] ) );
}

void traction_reset( void )
{
    reset_wheel( 0U );
    reset_wheel( 1U );
    s_front_seeded = false;
}

void traction_apply( const TelemetrySnapshot_t *data, int16_t torque[ 2 ], float dt_s )
{
    float front[ 2 ];
    float rear[ 2 ];
    float target_slip;
    const TcCutLevel_t *cut_level;
    uint32_t n;

    /* TC level 0 (off), or no ground speed reference -> can't compute
     * slip, fade any cut out. */
    if( ( data->tc_level == 0U ) || !data->front_axle_valid )
    {
        release_wheel( 0U, torque, dt_s );
        release_wheel( 1U, torque, dt_s );
        s_front_seeded = false;
        return;
    }

    /* TEST: FR sensor reads wrong, so FL is the ground speed for both
     * sides. Once FR is fixed, filter each front on its own and use
     * data->front_axle.wheel_speed_fr for front[ 1 ]. */
    if( !s_front_seeded )
    {
        s_front_f = ( float ) data->front_axle.wheel_speed_fl;
        s_front_seeded = true;
    }
    else
    {
        s_front_f += ( ( float ) data->front_axle.wheel_speed_fl - s_front_f ) * dt_s / ( TC_FRONT_TAU_S + dt_s );
    }
    target_slip = s_tc_target_slip[ ( data->tc_level > TC_LEVEL_MAX ) ? TC_LEVEL_MAX : data->tc_level ];
    cut_level = &s_tc_cut[ ( data->tc_cut_level < TC_CUT_LEVEL_MIN ) ? TC_CUT_LEVEL_MIN :
                           ( data->tc_cut_level > TC_CUT_LEVEL_MAX ) ? TC_CUT_LEVEL_MAX :
                           data->tc_cut_level ];

    front[ 0 ] = s_front_f;
    front[ 1 ] = s_front_f;
    rear[ 0 ]  = ( float ) data->rear.wheel_speed_rl;
    rear[ 1 ]  = ( float ) data->rear.wheel_speed_rr;

    for( n = 0U; n < 2U; n++ )
    {
        float error;
        float target;
        float ki;

        if( !data->driver[ n ].txpdo2_valid || ( rear[ n ] < TC_MIN_WHEEL_SPEED ) )
        {
            release_wheel( n, torque, dt_s );
            continue;
        }

        s_slip[ n ] = ( rear[ n ] - front[ n ] - TC_SLIP_SPEED_ALLOW ) / rear[ n ];
        error = s_slip[ n ] - target_slip;

        ki = ( error > 0.0f ) ? cut_level->ki : TC_KI_RELEASE;
        s_integral[ n ] = clamp( s_integral[ n ] + ( ki * error * dt_s ), 0.0f, cut_level->max_cut );
        target = clamp( ( cut_level->kp * error ) + s_integral[ n ], 0.0f, cut_level->max_cut );
        s_cut[ n ] = ramp_cut( s_cut[ n ], target, cut_level->rise_rate, dt_s );

        torque[ n ] = ( int16_t ) ( ( float ) torque[ n ] * ( 1.0f - s_cut[ n ] ) );
    }
}
