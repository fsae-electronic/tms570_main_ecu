#include "traction.h"

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
static float ramp_cut( float cut, float target, float dt_s )
{
    float max_up = TC_CUT_RISE_RATE * dt_s;
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
    s_cut[ n ] = ramp_cut( s_cut[ n ], 0.0f, dt_s );

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
    uint32_t n;

    /* No ground speed reference -> can't compute slip, fade any cut out. */
    if( !data->front_axle_valid )
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
        error = s_slip[ n ] - TC_TARGET_SLIP;

        ki = ( error > 0.0f ) ? TC_KI : TC_KI_RELEASE;
        s_integral[ n ] = clamp( s_integral[ n ] + ( ki * error * dt_s ), 0.0f, TC_MAX_CUT );
        target = clamp( ( TC_KP * error ) + s_integral[ n ], 0.0f, TC_MAX_CUT );
        s_cut[ n ] = ramp_cut( s_cut[ n ], target, dt_s );

        torque[ n ] = ( int16_t ) ( ( float ) torque[ n ] * ( 1.0f - s_cut[ n ] ) );
    }
}
