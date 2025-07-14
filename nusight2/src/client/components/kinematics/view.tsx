import React, { PropsWithChildren, useState, useRef, useEffect } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { KinematicsController } from "./controller";
import { KinematicsModel, ServoNames, getErrorDescription } from "./model";
import { CanvasWrapper } from "./r3f_components/canvas_wrapper";
import { KinematicsRobotModel } from "./robot_model";

const ServoDataDisplay: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  const [unit, setUnit] = useState<"rad" | "deg">("rad");
  const [isAlarmPlaying, setIsAlarmPlaying] = useState(false);
  const [audioStatus, setAudioStatus] = useState<string>("Initializing...");
  const [alarmEnabled, setAlarmEnabled] = useState<boolean>(() => {
    // Load alarm setting from localStorage, default to enabled
    const saved = localStorage.getItem('servoAlarmEnabled');
    return saved === null ? true : saved === 'true';
  });
  const audioContextRef = useRef<AudioContext | null>(null);
  const audioElementRef = useRef<HTMLAudioElement | null>(null);
  const alarmIntervalRef = useRef<number | null>(null);

  // Initialize multiple audio methods
  useEffect(() => {
    let audioInitialized = false;

    const initWebAudio = async () => {
      try {
        const AudioContextClass = window.AudioContext || (window as any).webkitAudioContext;
        audioContextRef.current = new AudioContextClass();

        if (audioContextRef.current.state === 'suspended') {
          await audioContextRef.current.resume();
        }

        console.log('Web Audio API initialized successfully');
        audioInitialized = true;
        setAudioStatus("Audio Ready");
        return true;
      } catch (error) {
        console.warn('Web Audio API failed:', error);
        return false;
      }
    };

    const initHTML5Audio = () => {
      try {
        const audio = new Audio();
        // Create a simple beep using data URL
        const beepDataUrl = 'data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT';
        audio.src = beepDataUrl;
        audio.volume = 0.5;
        audio.preload = 'auto';

        audioElementRef.current = audio;
        console.log('HTML5 Audio initialized successfully');
        audioInitialized = true;
        setAudioStatus("Audio Ready");
        return true;
      } catch (error) {
        console.warn('HTML5 Audio failed:', error);
        return false;
      }
    };

    const initAudio = async () => {
      // Try Web Audio API first
      const webAudioSuccess = await initWebAudio();

      // If Web Audio fails, try HTML5 Audio
      if (!webAudioSuccess) {
        initHTML5Audio();
      }
    };

    // Initialize on mount
    initAudio();

    // Set up user interaction handlers to resume audio
    const handleUserInteraction = async () => {
      if (!audioInitialized) {
        await initAudio();
      }

      if (audioContextRef.current?.state === 'suspended') {
        try {
          await audioContextRef.current.resume();
          console.log('Audio context resumed');
        } catch (error) {
          console.warn('Failed to resume audio context:', error);
        }
      }
    };

    // Add multiple event listeners for user interaction
    const events = ['click', 'keydown', 'touchstart', 'mousedown', 'focus'];
    events.forEach(event => {
      document.addEventListener(event, handleUserInteraction, { once: true });
    });

    return () => {
      events.forEach(event => {
        document.removeEventListener(event, handleUserInteraction);
      });

      if (alarmIntervalRef.current) {
        clearInterval(alarmIntervalRef.current);
      }
    };
  }, []);

  // Create warning alarm using Web Audio API
  const createWebAudioBeep = () => {
    if (!audioContextRef.current) return false;

    try {
      const now = audioContextRef.current.currentTime;

      // Create alternating high and low tones for warning effect
      const highFreq = 1200; // Higher frequency for urgency
      const lowFreq = 800;   // Lower frequency for contrast

      // First tone (high)
      const osc1 = audioContextRef.current.createOscillator();
      const gain1 = audioContextRef.current.createGain();
      osc1.connect(gain1);
      gain1.connect(audioContextRef.current.destination);

      osc1.frequency.setValueAtTime(highFreq, now);
      osc1.type = 'square'; // Square wave for more harsh warning sound

      gain1.gain.setValueAtTime(0, now);
      gain1.gain.linearRampToValueAtTime(0.2, now + 0.05);
      gain1.gain.linearRampToValueAtTime(0, now + 0.2);

      osc1.start(now);
      osc1.stop(now + 0.2);

      // Second tone (low) - starts after first tone
      const osc2 = audioContextRef.current.createOscillator();
      const gain2 = audioContextRef.current.createGain();
      osc2.connect(gain2);
      gain2.connect(audioContextRef.current.destination);

      osc2.frequency.setValueAtTime(lowFreq, now + 0.25);
      osc2.type = 'square';

      gain2.gain.setValueAtTime(0, now + 0.25);
      gain2.gain.linearRampToValueAtTime(0.2, now + 0.3);
      gain2.gain.linearRampToValueAtTime(0, now + 0.45);

      osc2.start(now + 0.25);
      osc2.stop(now + 0.45);

      return true;
    } catch (error) {
      console.warn('Web Audio alarm failed:', error);
      return false;
    }
  };

  // Create warning alarm using HTML5 Audio
  const createHTML5Beep = () => {
    if (!audioElementRef.current) return false;

    try {
      // Create a more warning-like sound with multiple tones
      const warningDataUrl = 'data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT';
      audioElementRef.current.src = warningDataUrl;
      audioElementRef.current.volume = 0.4;
      audioElementRef.current.currentTime = 0;
      audioElementRef.current.play().catch(error => {
        console.warn('HTML5 Audio play failed:', error);
      });
      return true;
    } catch (error) {
      console.warn('HTML5 Audio alarm failed:', error);
      return false;
    }
  };

  // Create warning alarm using system beep (fallback)
  const createSystemBeep = () => {
    try {
      // Try to use system beep if available
      if (typeof window !== 'undefined' && (window as any).beep) {
        (window as any).beep();
        return true;
      }

      // Fallback: try to create a warning sound with multiple tones
      const tempContext = new (window.AudioContext || (window as any).webkitAudioContext)();

      // Create alternating tones for warning effect
      const osc1 = tempContext.createOscillator();
      const gain1 = tempContext.createGain();
      const osc2 = tempContext.createOscillator();
      const gain2 = tempContext.createGain();

      osc1.connect(gain1);
      gain1.connect(tempContext.destination);
      osc2.connect(gain2);
      gain2.connect(tempContext.destination);

      // High tone
      osc1.frequency.value = 1200;
      gain1.gain.value = 0.15;
      osc1.start();
      osc1.stop(tempContext.currentTime + 0.2);

      // Low tone (delayed)
      osc2.frequency.value = 800;
      gain2.gain.value = 0.15;
      osc2.start(tempContext.currentTime + 0.25);
      osc2.stop(tempContext.currentTime + 0.45);

      return true;
    } catch (error) {
      console.warn('System alarm failed:', error);
      return false;
    }
  };

  // Unified warning alarm function that tries multiple methods
  const createBeep = () => {
    console.log('Attempting to create warning alarm...');

    // Try Web Audio API first
    if (createWebAudioBeep()) {
      console.log('Warning alarm created with Web Audio API');
      return true;
    }

    // Try HTML5 Audio
    if (createHTML5Beep()) {
      console.log('Warning alarm created with HTML5 Audio');
      return true;
    }

    // Try system beep as last resort
    if (createSystemBeep()) {
      console.log('Warning alarm created with system beep');
      return true;
    }

    console.warn('All audio methods failed');
    return false;
  };

  // Toggle alarm function
  const toggleAlarm = React.useCallback(() => {
    const newState = !alarmEnabled;
    setAlarmEnabled(newState);
    localStorage.setItem('servoAlarmEnabled', newState.toString());

    // If disabling alarm while it's playing, stop it
    if (!newState && isAlarmPlaying) {
      stopAlarm();
    }

    console.log(`Alarm ${newState ? 'enabled' : 'disabled'}`);
  }, [alarmEnabled, isAlarmPlaying]);

  // Audio alarm function
  const playAlarm = React.useCallback(() => {
    if (isAlarmPlaying || !alarmEnabled) return;

    console.log('Starting alarm...');
    setIsAlarmPlaying(true);

    // Play initial beep
    createBeep();

    // Set up repeating warning alarm every 1.5 seconds for urgency
    alarmIntervalRef.current = window.setInterval(() => {
      if (!robot.hasErrors || !alarmEnabled) {
        if (alarmIntervalRef.current) {
          clearInterval(alarmIntervalRef.current);
          alarmIntervalRef.current = null;
        }
        setIsAlarmPlaying(false);
        console.log('Warning alarm stopped - no errors or alarm disabled');
        return;
      }

      createBeep();
    }, 1500);

  }, [isAlarmPlaying, robot.hasErrors, alarmEnabled]);

  // Stop alarm when errors clear
  const stopAlarm = React.useCallback(() => {
    if (alarmIntervalRef.current) {
      clearInterval(alarmIntervalRef.current);
      alarmIntervalRef.current = null;
    }
    setIsAlarmPlaying(false);
    console.log('Alarm stopped manually');
  }, []);

  // Trigger alarm when errors are detected or cleared
  React.useEffect(() => {
    if (robot.hasErrors && !isAlarmPlaying && alarmEnabled) {
      console.log('Servo errors detected, starting alarm');
      playAlarm();
    } else if ((!robot.hasErrors || !alarmEnabled) && isAlarmPlaying) {
      console.log('Servo errors cleared or alarm disabled, stopping alarm');
      stopAlarm();
    }
  }, [robot.hasErrors, isAlarmPlaying, alarmEnabled, playAlarm, stopAlarm]);

  return (
    <div className="p-4 border border-black dark:border-white rounded-lg w-full">
      <div className="flex justify-between items-center mb-4 pb-2">
        <h3 className="text-xl font-semibold">Servo Information</h3>
        <div className="flex items-center gap-2">
          {robot.hasErrors && (
            <div className="flex items-center gap-1 text-red-600">
              <div className="w-2 h-2 bg-red-600 rounded-full animate-pulse"></div>
              <span className="text-xs font-medium">ALARM</span>
            </div>
          )}
          <div className="text-xs text-gray-500">{audioStatus}</div>
          <button
            onClick={toggleAlarm}
            className={`text-xs px-2 py-1 rounded mr-2 ${alarmEnabled
                ? 'bg-red-600 text-white hover:bg-red-700'
                : 'bg-gray-400 text-white hover:bg-gray-500'
              }`}
          >
            {alarmEnabled ? 'Disable Alarm' : 'Enable Alarm'}
          </button>
          <button
            onClick={() => {
              console.log('Testing audio...');
              createBeep();
            }}
            className="text-xs px-2 py-1 bg-green-600 text-white rounded hover:bg-green-700 mr-2"
          >
            Test Audio
          </button>
          <button
            onClick={() => setUnit(unit === "rad" ? "deg" : "rad")}
            className="text-sm px-2 py-1 border rounded hover:bg-gray-300 dark:hover:bg-gray-600 border-black dark:border-white"
          >
            Show in {unit === "rad" ? "Degrees" : "Radians"}
          </button>
        </div>
      </div>

      {/* Error Summary */}
      {robot.hasErrors && (
        <div className="mb-4 p-3 bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 rounded-lg">
          <h4 className="text-sm font-medium text-red-800 dark:text-red-200 mb-2">Servo Errors Detected</h4>
          <div className="space-y-1">
            {robot.servosWithErrors.map((servo) => (
              <div key={servo.id} className="text-xs text-red-700 dark:text-red-300">
                <span className="font-medium">{servo.name}:</span> {getErrorDescription(servo.error).join(", ")}
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Temperature Summary */}
      <div className="mb-4 grid grid-cols-2 gap-4">
        <div className="bg-gray-100 dark:bg-gray-800 p-3 rounded-lg">
          <h4 className="text-sm font-medium text-gray-600 dark:text-gray-400">Highest Temperature</h4>
          {robot.highestTemperatureServo && (
            <div className="text-lg font-bold">
              <span className={robot.highestTemperatureServo.temperature > 50 ? "text-red-600" : "text-[#888888]"}>
                {robot.highestTemperatureServo.temperature.toFixed(1)}°C
              </span>
              <div className="text-xs text-gray-500">{robot.highestTemperatureServo.name}</div>
            </div>
          )}
        </div>
        <div className="bg-gray-100 dark:bg-gray-800 p-3 rounded-lg">
          <h4 className="text-sm font-medium text-gray-600 dark:text-gray-400">Average Temperature</h4>
          <div className="text-lg font-bold text-[#888888]">{robot.averageTemperature.toFixed(1)}°C</div>
        </div>
      </div>

      {/* Servo Table */}
      <div className="overflow-x-auto">
        <table className="w-full text-sm">
          <thead>
            <tr className="border-b border-gray-300 dark:border-gray-600">
              <th className="text-left p-2 font-medium">Servo</th>
              <th className="text-right p-2 font-medium">Angle</th>
              <th className="text-right p-2 font-medium">Temperature</th>
              <th className="text-right p-2 font-medium">Status</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
            {Object.entries(robot.motors).map(([jointName, motor], index) => {
              const servoId = index;
              const servoName = ServoNames[servoId] || jointName.replace(/([a-z])([A-Z])/g, "$1 $2").replace(/^./, (match) => match.toUpperCase());
              const temperature = robot.servoTemperatures.get(servoId);
              const error = robot.servoErrors.get(servoId);
              const angle = unit === "rad" ? `${motor.angle.toFixed(2)} rad` : `${((motor.angle * 180) / Math.PI).toFixed(2)}°`;
              const isOverLimit = temperature !== undefined && temperature > 50;
              const hasError = error !== undefined && error !== 0;

              return (
                <tr
                  key={jointName}
                  className={`hover:bg-gray-50 dark:hover:bg-gray-800 ${isOverLimit || hasError ? 'bg-red-50 dark:bg-red-900/20' : ''
                    }`}
                >
                  <td className="p-2 font-medium">{servoName}</td>
                  <td className="p-2 text-right">{angle}</td>
                  <td className="p-2 text-right">
                    {temperature !== undefined ? (
                      <span className={isOverLimit ? "text-red-600 font-medium" : "text-[#888888]"}>
                        {temperature.toFixed(1)}°C
                      </span>
                    ) : (
                      <span className="text-gray-400">-</span>
                    )}
                  </td>
                  <td className="p-2 text-right">
                    {isOverLimit ? (
                      <span className="text-red-600 font-medium text-xs">HOT</span>
                    ) : hasError ? (
                      <span className="text-red-600 font-medium text-xs">
                        {getErrorDescription(error!)[0]}
                      </span>
                    ) : (
                      <span className="text-green-600 text-xs">OK</span>
                    )}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
});

@observer
export class KinematicsView extends React.Component<{
  controller: KinematicsController;
  model: KinematicsModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    return (
      <div className="w-full h-screen flex flex-col">
        <Menu>
          <div className="h-full flex items-center justify-end">
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>

        {selectedRobot && (
          <div className="flex flex-1 overflow-hidden">
            <div className="flex-1 relative">
              <CanvasWrapper selectedRobot={selectedRobot} />
            </div>

            <div className="w-1/3 h-full overflow-y-auto">
              <ServoDataDisplay robot={selectedRobot} />
            </div>
          </div>
        )}
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
