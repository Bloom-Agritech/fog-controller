#define NUM_STATES 6


int idle_state(void);
int set_vpd_setpoint_state(void);
int set_vpd_deadband_state(void);
int set_duty_cycle_state(void);
int set_period_state(void);
int set_light_threshold_state(void);

int (* state[])(void) {
    idle_state,
    set_vpd_setpoint_state,
    set_vpd_deadband_state,
    set_duty_cycle_state,
    set_period_state,
    set_light_threshold_state
};

enum state_codes {
  IDLE,
  SET_VPD_SETPOINT,
  SET_VPD_DEADBAND,
  SET_DUTY_CYCLE,
  SET_PERIOD,
  SET_LIGHT_THRESHOLD,
  LAST
};

enum ret_codes {NEXT, TIMEOUT};

struct transition {
  enum state_codes current_state;
  enum ret_codes ret_code;
  enum state_codes next_state; 
};

struct transition state_transitions[] = {
  // NEXT
  {IDLE, NEXT, SET_VPD_SETPOINT},
  {SET_VPD_SETPOINT, NEXT, SET_VPD_DEADBAND},
  {SET_VPD_DEADBAND, NEXT, SET_DUTY_CYCLE},
  {SET_DUTY_CYCLE, NEXT, SET_PERIOD},
  {SET_PERIOD, NEXT, SET_LIGHT_THRESHOLD},
  {SET_LIGHT_THRESHOLD, NEXT, IDLE},
  // TIMEOUT
  {SET_VPD_SETPOINT, TIMEOUT, IDLE},
  {SET_VPD_DEADBAND, TIMEOUT, IDLE},
  {SET_DUTY_CYCLE, TIMEOUT, IDLE},
  {SET_PERIOD, TIMEOUT, IDLE},
  {SET_LIGHT_THRESHOLD, TIMEOUT, IDLE}
};

