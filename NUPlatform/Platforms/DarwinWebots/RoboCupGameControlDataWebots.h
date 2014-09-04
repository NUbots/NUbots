#ifndef RoboCupGameControlDataWebots_H
#define RoboCupGameControlDataWebots_H

// information that describes a team
struct TeamInfoWebots {
  uint8 teamNumber;                           // team number 0 or 1
  uint8 teamColour;                           // TEAM_BLUE or TEAM_RED
  uint16 score;                               // team's current score
  struct RobotInfo players[4];                // team's robots
};

// warning: this structure must be aligned on 32 bit words: modify with care !
struct RoboCupGameControlDataWebots {
  // subset of the original RoboCupGameControlData fields:
  char header[4];             // header to identify the structure
  uint32 version;             // version of the data structure
  uint8 playersPerTeam;       // The number of players on a team
  uint8 state;                // state of the game (STATE_READY, STATE_PLAYING, etc)
  uint8 firstHalf;            // 1 = game in first half, 0 otherwise
  uint8 kickOffTeam;          // the next team to kick off
  uint8 secondaryState;       // Extra state information (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  uint8 dropInTeam;           // Team that caused last drop in (not supported in Robotstadium)
  uint16 dropInTime;          // Number of seconds passed since the last drop in (not supported in Robotstadium)
  uint32 secsRemaining;       // estimate of number of seconds remaining in the half
  struct TeamInfoWebots teams[2];   // blue team first, red team last

  // Webots extra: global ball position
  // FOR TRAINING ONLY: THIS WILL BE DISABLED IN THE CONTEST MATCHES !!!
  float ballXPos;
  float ballZPos;
};

#endif
