FSM(Millenial_Falcon)
{
    FSM_STATES
    {
        TAKEOFF
        SEARCH
        LANDING
    }
    FSM_START(TAKEOFF);
    FSM_BGN
    {
         FSM_STATE(TAKEOFF)
         {
              FSM_TRANSITIONS
              {
                  //Altitude Boolean is a 1 when we have reached altitude to begin search
                  //Altitude Boolean is a 0 when we have not reached altitude
                  FSM_ON_CONDITION(ALTITUDEBOOL == 1, FSM_NEXT(SEARCH));
               }
          }
          FSM_STATE(SEARCH)
          {
              FSM_TRANSITIONS
              {
                //LandBool is 1 when num beacons found =1 or battery level <= battery required for landing
                FSM_ON_CONDITION(BeaconsFound == 1 | LandingBatteryCrit == 1, FSM_NEXT(LANDING))
              }
          }
          FSM_STATE(LANDING)
          {

          }
    }
    FSM_END
