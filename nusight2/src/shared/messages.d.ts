import * as $protobuf from "protobufjs";
/** Namespace message. */
export namespace message {

    /** Namespace actuation. */
    namespace actuation {

        /** BodySide enum. */
        enum BodySide {
            LEFT = 0,
            RIGHT = 1
        }

        /** Properties of a KinematicsModel. */
        interface IKinematicsModel {

            /** KinematicsModel leg */
            leg?: (message.actuation.KinematicsModel.ILeg|null);

            /** KinematicsModel head */
            head?: (message.actuation.KinematicsModel.IHead|null);

            /** KinematicsModel arm */
            arm?: (message.actuation.KinematicsModel.IArm|null);

            /** KinematicsModel massModel */
            massModel?: (message.actuation.KinematicsModel.IMassModel|null);

            /** KinematicsModel tensorModel */
            tensorModel?: (message.actuation.KinematicsModel.ITensorModel|null);
        }

        /**
         * Contains lots of kinematics config measurements and transforms.
         * See `KinematicsConfiguration.yaml` for more detail on a particular variable.
         * Convention: all values positive
         */
        class KinematicsModel implements IKinematicsModel {

            /**
             * Constructs a new KinematicsModel.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IKinematicsModel);

            /** KinematicsModel leg. */
            public leg?: (message.actuation.KinematicsModel.ILeg|null);

            /** KinematicsModel head. */
            public head?: (message.actuation.KinematicsModel.IHead|null);

            /** KinematicsModel arm. */
            public arm?: (message.actuation.KinematicsModel.IArm|null);

            /** KinematicsModel massModel. */
            public massModel?: (message.actuation.KinematicsModel.IMassModel|null);

            /** KinematicsModel tensorModel. */
            public tensorModel?: (message.actuation.KinematicsModel.ITensorModel|null);

            /**
             * Encodes the specified KinematicsModel message. Does not implicitly {@link message.actuation.KinematicsModel.verify|verify} messages.
             * @param message KinematicsModel message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IKinematicsModel, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KinematicsModel message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KinematicsModel
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel;
        }

        namespace KinematicsModel {

            /** Properties of a Leg. */
            interface ILeg {

                /** Leg HIP_OFFSET_X */
                HIP_OFFSET_X?: (number|null);

                /** Leg HIP_OFFSET_Y */
                HIP_OFFSET_Y?: (number|null);

                /** Leg HIP_OFFSET_Z */
                HIP_OFFSET_Z?: (number|null);

                /** Leg UPPER_LEG_LENGTH */
                UPPER_LEG_LENGTH?: (number|null);

                /** Leg LOWER_LEG_LENGTH */
                LOWER_LEG_LENGTH?: (number|null);

                /** Leg FOOT_HEIGHT */
                FOOT_HEIGHT?: (number|null);

                /** Leg FOOT_LENGTH */
                FOOT_LENGTH?: (number|null);

                /** Leg TOE_LENGTH */
                TOE_LENGTH?: (number|null);

                /** Leg HEEL_LENGTH */
                HEEL_LENGTH?: (number|null);

                /** Leg FOOT_WIDTH */
                FOOT_WIDTH?: (number|null);

                /** Leg FOOT_CENTRE_TO_ANKLE_CENTRE */
                FOOT_CENTRE_TO_ANKLE_CENTRE?: (number|null);

                /** Leg LENGTH_BETWEEN_LEGS */
                LENGTH_BETWEEN_LEGS?: (number|null);

                /** Leg LEFT_TO_RIGHT_HIP_YAW */
                LEFT_TO_RIGHT_HIP_YAW?: (number|null);

                /** Leg LEFT_TO_RIGHT_HIP_ROLL */
                LEFT_TO_RIGHT_HIP_ROLL?: (number|null);

                /** Leg LEFT_TO_RIGHT_HIP_PITCH */
                LEFT_TO_RIGHT_HIP_PITCH?: (number|null);

                /** Leg LEFT_TO_RIGHT_KNEE */
                LEFT_TO_RIGHT_KNEE?: (number|null);

                /** Leg LEFT_TO_RIGHT_ANKLE_PITCH */
                LEFT_TO_RIGHT_ANKLE_PITCH?: (number|null);

                /** Leg LEFT_TO_RIGHT_ANKLE_ROLL */
                LEFT_TO_RIGHT_ANKLE_ROLL?: (number|null);
            }

            /** Represents a Leg. */
            class Leg implements ILeg {

                /**
                 * Constructs a new Leg.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.actuation.KinematicsModel.ILeg);

                /** Leg HIP_OFFSET_X. */
                public HIP_OFFSET_X: number;

                /** Leg HIP_OFFSET_Y. */
                public HIP_OFFSET_Y: number;

                /** Leg HIP_OFFSET_Z. */
                public HIP_OFFSET_Z: number;

                /** Leg UPPER_LEG_LENGTH. */
                public UPPER_LEG_LENGTH: number;

                /** Leg LOWER_LEG_LENGTH. */
                public LOWER_LEG_LENGTH: number;

                /** Leg FOOT_HEIGHT. */
                public FOOT_HEIGHT: number;

                /** Leg FOOT_LENGTH. */
                public FOOT_LENGTH: number;

                /** Leg TOE_LENGTH. */
                public TOE_LENGTH: number;

                /** Leg HEEL_LENGTH. */
                public HEEL_LENGTH: number;

                /** Leg FOOT_WIDTH. */
                public FOOT_WIDTH: number;

                /** Leg FOOT_CENTRE_TO_ANKLE_CENTRE. */
                public FOOT_CENTRE_TO_ANKLE_CENTRE: number;

                /** Leg LENGTH_BETWEEN_LEGS. */
                public LENGTH_BETWEEN_LEGS: number;

                /** Leg LEFT_TO_RIGHT_HIP_YAW. */
                public LEFT_TO_RIGHT_HIP_YAW: number;

                /** Leg LEFT_TO_RIGHT_HIP_ROLL. */
                public LEFT_TO_RIGHT_HIP_ROLL: number;

                /** Leg LEFT_TO_RIGHT_HIP_PITCH. */
                public LEFT_TO_RIGHT_HIP_PITCH: number;

                /** Leg LEFT_TO_RIGHT_KNEE. */
                public LEFT_TO_RIGHT_KNEE: number;

                /** Leg LEFT_TO_RIGHT_ANKLE_PITCH. */
                public LEFT_TO_RIGHT_ANKLE_PITCH: number;

                /** Leg LEFT_TO_RIGHT_ANKLE_ROLL. */
                public LEFT_TO_RIGHT_ANKLE_ROLL: number;

                /**
                 * Encodes the specified Leg message. Does not implicitly {@link message.actuation.KinematicsModel.Leg.verify|verify} messages.
                 * @param message Leg message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.actuation.KinematicsModel.ILeg, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Leg message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Leg
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel.Leg;
            }

            /** Properties of a Head. */
            interface IHead {

                /** Head NECK_BASE_POS_FROM_ORIGIN_X */
                NECK_BASE_POS_FROM_ORIGIN_X?: (number|null);

                /** Head NECK_BASE_POS_FROM_ORIGIN_Y */
                NECK_BASE_POS_FROM_ORIGIN_Y?: (number|null);

                /** Head NECK_BASE_POS_FROM_ORIGIN_Z */
                NECK_BASE_POS_FROM_ORIGIN_Z?: (number|null);

                /** Head NECK_LENGTH */
                NECK_LENGTH?: (number|null);

                /** Head NECK_TO_CAMERA_X */
                NECK_TO_CAMERA_X?: (number|null);

                /** Head NECK_TO_CAMERA_Y */
                NECK_TO_CAMERA_Y?: (number|null);

                /** Head NECK_TO_CAMERA_Z */
                NECK_TO_CAMERA_Z?: (number|null);

                /** Head CAMERA_DECLINATION_ANGLE_OFFSET */
                CAMERA_DECLINATION_ANGLE_OFFSET?: (number|null);

                /** Head INTERPUPILLARY_DISTANCE */
                INTERPUPILLARY_DISTANCE?: (number|null);

                /** Head MAX_YAW */
                MAX_YAW?: (number|null);

                /** Head MIN_YAW */
                MIN_YAW?: (number|null);

                /** Head MAX_PITCH */
                MAX_PITCH?: (number|null);

                /** Head MIN_PITCH */
                MIN_PITCH?: (number|null);
            }

            /** Represents a Head. */
            class Head implements IHead {

                /**
                 * Constructs a new Head.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.actuation.KinematicsModel.IHead);

                /** Head NECK_BASE_POS_FROM_ORIGIN_X. */
                public NECK_BASE_POS_FROM_ORIGIN_X: number;

                /** Head NECK_BASE_POS_FROM_ORIGIN_Y. */
                public NECK_BASE_POS_FROM_ORIGIN_Y: number;

                /** Head NECK_BASE_POS_FROM_ORIGIN_Z. */
                public NECK_BASE_POS_FROM_ORIGIN_Z: number;

                /** Head NECK_LENGTH. */
                public NECK_LENGTH: number;

                /** Head NECK_TO_CAMERA_X. */
                public NECK_TO_CAMERA_X: number;

                /** Head NECK_TO_CAMERA_Y. */
                public NECK_TO_CAMERA_Y: number;

                /** Head NECK_TO_CAMERA_Z. */
                public NECK_TO_CAMERA_Z: number;

                /** Head CAMERA_DECLINATION_ANGLE_OFFSET. */
                public CAMERA_DECLINATION_ANGLE_OFFSET: number;

                /** Head INTERPUPILLARY_DISTANCE. */
                public INTERPUPILLARY_DISTANCE: number;

                /** Head MAX_YAW. */
                public MAX_YAW: number;

                /** Head MIN_YAW. */
                public MIN_YAW: number;

                /** Head MAX_PITCH. */
                public MAX_PITCH: number;

                /** Head MIN_PITCH. */
                public MIN_PITCH: number;

                /**
                 * Encodes the specified Head message. Does not implicitly {@link message.actuation.KinematicsModel.Head.verify|verify} messages.
                 * @param message Head message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.actuation.KinematicsModel.IHead, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Head message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Head
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel.Head;
            }

            /** Properties of an Arm. */
            interface IArm {

                /** Arm DISTANCE_BETWEEN_SHOULDERS */
                DISTANCE_BETWEEN_SHOULDERS?: (number|null);

                /** Arm SHOULDER_Z_OFFSET */
                SHOULDER_Z_OFFSET?: (number|null);

                /** Arm SHOULDER_X_OFFSET */
                SHOULDER_X_OFFSET?: (number|null);

                /** Arm SHOULDER_LENGTH */
                SHOULDER_LENGTH?: (number|null);

                /** Arm SHOULDER_WIDTH */
                SHOULDER_WIDTH?: (number|null);

                /** Arm SHOULDER_HEIGHT */
                SHOULDER_HEIGHT?: (number|null);

                /** Arm UPPER_ARM_LENGTH */
                UPPER_ARM_LENGTH?: (number|null);

                /** Arm UPPER_ARM_Y_OFFSET */
                UPPER_ARM_Y_OFFSET?: (number|null);

                /** Arm UPPER_ARM_X_OFFSET */
                UPPER_ARM_X_OFFSET?: (number|null);

                /** Arm LOWER_ARM_LENGTH */
                LOWER_ARM_LENGTH?: (number|null);

                /** Arm LOWER_ARM_Y_OFFSET */
                LOWER_ARM_Y_OFFSET?: (number|null);

                /** Arm LOWER_ARM_Z_OFFSET */
                LOWER_ARM_Z_OFFSET?: (number|null);
            }

            /** Represents an Arm. */
            class Arm implements IArm {

                /**
                 * Constructs a new Arm.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.actuation.KinematicsModel.IArm);

                /** Arm DISTANCE_BETWEEN_SHOULDERS. */
                public DISTANCE_BETWEEN_SHOULDERS: number;

                /** Arm SHOULDER_Z_OFFSET. */
                public SHOULDER_Z_OFFSET: number;

                /** Arm SHOULDER_X_OFFSET. */
                public SHOULDER_X_OFFSET: number;

                /** Arm SHOULDER_LENGTH. */
                public SHOULDER_LENGTH: number;

                /** Arm SHOULDER_WIDTH. */
                public SHOULDER_WIDTH: number;

                /** Arm SHOULDER_HEIGHT. */
                public SHOULDER_HEIGHT: number;

                /** Arm UPPER_ARM_LENGTH. */
                public UPPER_ARM_LENGTH: number;

                /** Arm UPPER_ARM_Y_OFFSET. */
                public UPPER_ARM_Y_OFFSET: number;

                /** Arm UPPER_ARM_X_OFFSET. */
                public UPPER_ARM_X_OFFSET: number;

                /** Arm LOWER_ARM_LENGTH. */
                public LOWER_ARM_LENGTH: number;

                /** Arm LOWER_ARM_Y_OFFSET. */
                public LOWER_ARM_Y_OFFSET: number;

                /** Arm LOWER_ARM_Z_OFFSET. */
                public LOWER_ARM_Z_OFFSET: number;

                /**
                 * Encodes the specified Arm message. Does not implicitly {@link message.actuation.KinematicsModel.Arm.verify|verify} messages.
                 * @param message Arm message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.actuation.KinematicsModel.IArm, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an Arm message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Arm
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel.Arm;
            }

            /** Properties of a MassModel. */
            interface IMassModel {

                /** MassModel head */
                head?: (Ivec4|null);

                /** MassModel armUpper */
                armUpper?: (Ivec4|null);

                /** MassModel armLower */
                armLower?: (Ivec4|null);

                /** MassModel torso */
                torso?: (Ivec4|null);

                /** MassModel hipBlock */
                hipBlock?: (Ivec4|null);

                /** MassModel legUpper */
                legUpper?: (Ivec4|null);

                /** MassModel legLower */
                legLower?: (Ivec4|null);

                /** MassModel ankleBlock */
                ankleBlock?: (Ivec4|null);

                /** MassModel foot */
                foot?: (Ivec4|null);
            }

            /** A Particles CoM is in particle space */
            class MassModel implements IMassModel {

                /**
                 * Constructs a new MassModel.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.actuation.KinematicsModel.IMassModel);

                /** MassModel head. */
                public head?: (Ivec4|null);

                /** MassModel armUpper. */
                public armUpper?: (Ivec4|null);

                /** MassModel armLower. */
                public armLower?: (Ivec4|null);

                /** MassModel torso. */
                public torso?: (Ivec4|null);

                /** MassModel hipBlock. */
                public hipBlock?: (Ivec4|null);

                /** MassModel legUpper. */
                public legUpper?: (Ivec4|null);

                /** MassModel legLower. */
                public legLower?: (Ivec4|null);

                /** MassModel ankleBlock. */
                public ankleBlock?: (Ivec4|null);

                /** MassModel foot. */
                public foot?: (Ivec4|null);

                /**
                 * Encodes the specified MassModel message. Does not implicitly {@link message.actuation.KinematicsModel.MassModel.verify|verify} messages.
                 * @param message MassModel message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.actuation.KinematicsModel.IMassModel, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MassModel message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MassModel
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel.MassModel;
            }

            /** Properties of a TensorModel. */
            interface ITensorModel {

                /** TensorModel head */
                head?: (Imat3|null);

                /** TensorModel armUpper */
                armUpper?: (Imat3|null);

                /** TensorModel armLower */
                armLower?: (Imat3|null);

                /** TensorModel torso */
                torso?: (Imat3|null);

                /** TensorModel hipBlock */
                hipBlock?: (Imat3|null);

                /** TensorModel legUpper */
                legUpper?: (Imat3|null);

                /** TensorModel legLower */
                legLower?: (Imat3|null);

                /** TensorModel ankleBlock */
                ankleBlock?: (Imat3|null);

                /** TensorModel foot */
                foot?: (Imat3|null);
            }

            /** SI Units */
            class TensorModel implements ITensorModel {

                /**
                 * Constructs a new TensorModel.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.actuation.KinematicsModel.ITensorModel);

                /** TensorModel head. */
                public head?: (Imat3|null);

                /** TensorModel armUpper. */
                public armUpper?: (Imat3|null);

                /** TensorModel armLower. */
                public armLower?: (Imat3|null);

                /** TensorModel torso. */
                public torso?: (Imat3|null);

                /** TensorModel hipBlock. */
                public hipBlock?: (Imat3|null);

                /** TensorModel legUpper. */
                public legUpper?: (Imat3|null);

                /** TensorModel legLower. */
                public legLower?: (Imat3|null);

                /** TensorModel ankleBlock. */
                public ankleBlock?: (Imat3|null);

                /** TensorModel foot. */
                public foot?: (Imat3|null);

                /**
                 * Encodes the specified TensorModel message. Does not implicitly {@link message.actuation.KinematicsModel.TensorModel.verify|verify} messages.
                 * @param message TensorModel message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.actuation.KinematicsModel.ITensorModel, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a TensorModel message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns TensorModel
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.KinematicsModel.TensorModel;
            }
        }

        /** Properties of a LeftLeg. */
        interface ILeftLeg {

            /** Information for each servo in the left leg, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Left leg servo command information including time, gain, torque, position */
        class LeftLeg implements ILeftLeg {

            /**
             * Constructs a new LeftLeg.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftLeg);

            /** Information for each servo in the left leg, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified LeftLeg message. Does not implicitly {@link message.actuation.LeftLeg.verify|verify} messages.
             * @param message LeftLeg message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftLeg, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftLeg message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftLeg
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftLeg;
        }

        /** Properties of a LeftLegSequence. */
        interface ILeftLegSequence {

            /** LeftLegSequence frames */
            frames?: (message.actuation.ILeftLeg[]|null);
        }

        /** Represents a LeftLegSequence. */
        class LeftLegSequence implements ILeftLegSequence {

            /**
             * Constructs a new LeftLegSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftLegSequence);

            /** LeftLegSequence frames. */
            public frames: message.actuation.ILeftLeg[];

            /**
             * Encodes the specified LeftLegSequence message. Does not implicitly {@link message.actuation.LeftLegSequence.verify|verify} messages.
             * @param message LeftLegSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftLegSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftLegSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftLegSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftLegSequence;
        }

        /** Properties of a RightLeg. */
        interface IRightLeg {

            /** Information for each servo in the right leg, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Right leg servo command information including time, gain, torque, position */
        class RightLeg implements IRightLeg {

            /**
             * Constructs a new RightLeg.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightLeg);

            /** Information for each servo in the right leg, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified RightLeg message. Does not implicitly {@link message.actuation.RightLeg.verify|verify} messages.
             * @param message RightLeg message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightLeg, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightLeg message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightLeg
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightLeg;
        }

        /** Properties of a RightLegSequence. */
        interface IRightLegSequence {

            /** RightLegSequence frames */
            frames?: (message.actuation.IRightLeg[]|null);
        }

        /** Represents a RightLegSequence. */
        class RightLegSequence implements IRightLegSequence {

            /**
             * Constructs a new RightLegSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightLegSequence);

            /** RightLegSequence frames. */
            public frames: message.actuation.IRightLeg[];

            /**
             * Encodes the specified RightLegSequence message. Does not implicitly {@link message.actuation.RightLegSequence.verify|verify} messages.
             * @param message RightLegSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightLegSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightLegSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightLegSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightLegSequence;
        }

        /** Properties of a Legs. */
        interface ILegs {

            /** Information for each servo in the legs, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Right and left leg servo command information including time, gain, torque, position */
        class Legs implements ILegs {

            /**
             * Constructs a new Legs.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILegs);

            /** Information for each servo in the legs, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified Legs message. Does not implicitly {@link message.actuation.Legs.verify|verify} messages.
             * @param message Legs message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILegs, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Legs message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Legs
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.Legs;
        }

        /** Properties of a LegsSequence. */
        interface ILegsSequence {

            /** LegsSequence frames */
            frames?: (message.actuation.ILegs[]|null);
        }

        /** Represents a LegsSequence. */
        class LegsSequence implements ILegsSequence {

            /**
             * Constructs a new LegsSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILegsSequence);

            /** LegsSequence frames. */
            public frames: message.actuation.ILegs[];

            /**
             * Encodes the specified LegsSequence message. Does not implicitly {@link message.actuation.LegsSequence.verify|verify} messages.
             * @param message LegsSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILegsSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LegsSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LegsSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LegsSequence;
        }

        /** Properties of a LeftArm. */
        interface ILeftArm {

            /** Information for each servo in the left arm, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Left arm servo command information including time, gain, torque, position */
        class LeftArm implements ILeftArm {

            /**
             * Constructs a new LeftArm.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftArm);

            /** Information for each servo in the left arm, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified LeftArm message. Does not implicitly {@link message.actuation.LeftArm.verify|verify} messages.
             * @param message LeftArm message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftArm, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftArm message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftArm
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftArm;
        }

        /** Properties of a LeftArmSequence. */
        interface ILeftArmSequence {

            /** LeftArmSequence frames */
            frames?: (message.actuation.ILeftArm[]|null);
        }

        /** Represents a LeftArmSequence. */
        class LeftArmSequence implements ILeftArmSequence {

            /**
             * Constructs a new LeftArmSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftArmSequence);

            /** LeftArmSequence frames. */
            public frames: message.actuation.ILeftArm[];

            /**
             * Encodes the specified LeftArmSequence message. Does not implicitly {@link message.actuation.LeftArmSequence.verify|verify} messages.
             * @param message LeftArmSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftArmSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftArmSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftArmSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftArmSequence;
        }

        /** Properties of a RightArm. */
        interface IRightArm {

            /** Information for each servo in the right arm, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Right arm servo command information including time, gain, torque, position */
        class RightArm implements IRightArm {

            /**
             * Constructs a new RightArm.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightArm);

            /** Information for each servo in the right arm, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified RightArm message. Does not implicitly {@link message.actuation.RightArm.verify|verify} messages.
             * @param message RightArm message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightArm, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightArm message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightArm
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightArm;
        }

        /** Properties of a RightArmSequence. */
        interface IRightArmSequence {

            /** RightArmSequence frames */
            frames?: (message.actuation.IRightArm[]|null);
        }

        /** Represents a RightArmSequence. */
        class RightArmSequence implements IRightArmSequence {

            /**
             * Constructs a new RightArmSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightArmSequence);

            /** RightArmSequence frames. */
            public frames: message.actuation.IRightArm[];

            /**
             * Encodes the specified RightArmSequence message. Does not implicitly {@link message.actuation.RightArmSequence.verify|verify} messages.
             * @param message RightArmSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightArmSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightArmSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightArmSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightArmSequence;
        }

        /** Properties of an Arms. */
        interface IArms {

            /** Information for each servo in the arms, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Left and right arm servo command information including time, gain, torque, position */
        class Arms implements IArms {

            /**
             * Constructs a new Arms.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IArms);

            /** Information for each servo in the arms, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified Arms message. Does not implicitly {@link message.actuation.Arms.verify|verify} messages.
             * @param message Arms message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IArms, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an Arms message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Arms
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.Arms;
        }

        /** Properties of an ArmsSequence. */
        interface IArmsSequence {

            /** ArmsSequence frames */
            frames?: (message.actuation.IArms[]|null);
        }

        /** Represents an ArmsSequence. */
        class ArmsSequence implements IArmsSequence {

            /**
             * Constructs a new ArmsSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IArmsSequence);

            /** ArmsSequence frames. */
            public frames: message.actuation.IArms[];

            /**
             * Encodes the specified ArmsSequence message. Does not implicitly {@link message.actuation.ArmsSequence.verify|verify} messages.
             * @param message ArmsSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IArmsSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an ArmsSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ArmsSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.ArmsSequence;
        }

        /** Properties of a Limbs. */
        interface ILimbs {

            /** Information for each servo in the arms and legs, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Left and right arm and leg servo command information including time, gain, torque, position */
        class Limbs implements ILimbs {

            /**
             * Constructs a new Limbs.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILimbs);

            /** Information for each servo in the arms and legs, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified Limbs message. Does not implicitly {@link message.actuation.Limbs.verify|verify} messages.
             * @param message Limbs message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILimbs, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Limbs message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Limbs
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.Limbs;
        }

        /** Properties of a LimbsSequence. */
        interface ILimbsSequence {

            /** LimbsSequence frames */
            frames?: (message.actuation.ILimbs[]|null);
        }

        /** Represents a LimbsSequence. */
        class LimbsSequence implements ILimbsSequence {

            /**
             * Constructs a new LimbsSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILimbsSequence);

            /** LimbsSequence frames. */
            public frames: message.actuation.ILimbs[];

            /**
             * Encodes the specified LimbsSequence message. Does not implicitly {@link message.actuation.LimbsSequence.verify|verify} messages.
             * @param message LimbsSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILimbsSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LimbsSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LimbsSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LimbsSequence;
        }

        /** Properties of a Head. */
        interface IHead {

            /** Information for each servo in the head, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Head servo command information including time, gain, torque, position */
        class Head implements IHead {

            /**
             * Constructs a new Head.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IHead);

            /** Information for each servo in the head, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified Head message. Does not implicitly {@link message.actuation.Head.verify|verify} messages.
             * @param message Head message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IHead, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Head message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Head
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.Head;
        }

        /** Properties of a HeadSequence. */
        interface IHeadSequence {

            /** HeadSequence frames */
            frames?: (message.actuation.IHead[]|null);
        }

        /** Represents a HeadSequence. */
        class HeadSequence implements IHeadSequence {

            /**
             * Constructs a new HeadSequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IHeadSequence);

            /** HeadSequence frames. */
            public frames: message.actuation.IHead[];

            /**
             * Encodes the specified HeadSequence message. Does not implicitly {@link message.actuation.HeadSequence.verify|verify} messages.
             * @param message HeadSequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IHeadSequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a HeadSequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns HeadSequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.HeadSequence;
        }

        /** Properties of a Body. */
        interface IBody {

            /** Information for each servo in the body, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoCommand }|null);
        }

        /** Arms, legs and head servo command information including time, gain, torque, position */
        class Body implements IBody {

            /**
             * Constructs a new Body.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IBody);

            /** Information for each servo in the body, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoCommand };

            /**
             * Encodes the specified Body message. Does not implicitly {@link message.actuation.Body.verify|verify} messages.
             * @param message Body message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IBody, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Body message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Body
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.Body;
        }

        /** Properties of a BodySequence. */
        interface IBodySequence {

            /** BodySequence frames */
            frames?: (message.actuation.IBody[]|null);
        }

        /** Represents a BodySequence. */
        class BodySequence implements IBodySequence {

            /**
             * Constructs a new BodySequence.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IBodySequence);

            /** BodySequence frames. */
            public frames: message.actuation.IBody[];

            /**
             * Encodes the specified BodySequence message. Does not implicitly {@link message.actuation.BodySequence.verify|verify} messages.
             * @param message BodySequence message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IBodySequence, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BodySequence message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BodySequence
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.BodySequence;
        }

        /** Properties of a LeftLegIK. */
        interface ILeftLegIK {

            /** When the command should execute */
            time?: (google.protobuf.ITimestamp|null);

            /** Target left foot position to torso */
            Htl?: (Imat4|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoState }|null);
        }

        /** Left leg motion information including target matrix, time and gain and torque for each servo */
        class LeftLegIK implements ILeftLegIK {

            /**
             * Constructs a new LeftLegIK.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftLegIK);

            /** When the command should execute */
            public time?: (google.protobuf.ITimestamp|null);

            /** Target left foot position to torso */
            public Htl?: (Imat4|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoState };

            /**
             * Encodes the specified LeftLegIK message. Does not implicitly {@link message.actuation.LeftLegIK.verify|verify} messages.
             * @param message LeftLegIK message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftLegIK, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftLegIK message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftLegIK
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftLegIK;
        }

        /** Properties of a RightLegIK. */
        interface IRightLegIK {

            /** When the command should execute */
            time?: (google.protobuf.ITimestamp|null);

            /** Target right foot position to torso */
            Htr?: (Imat4|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoState }|null);
        }

        /** Right leg motion information including target matrix, time and gain and torque for each servo */
        class RightLegIK implements IRightLegIK {

            /**
             * Constructs a new RightLegIK.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightLegIK);

            /** When the command should execute */
            public time?: (google.protobuf.ITimestamp|null);

            /** Target right foot position to torso */
            public Htr?: (Imat4|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoState };

            /**
             * Encodes the specified RightLegIK message. Does not implicitly {@link message.actuation.RightLegIK.verify|verify} messages.
             * @param message RightLegIK message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightLegIK, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightLegIK message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightLegIK
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightLegIK;
        }

        /** Properties of a HeadIK. */
        interface IHeadIK {

            /** When the command should execute */
            time?: (google.protobuf.ITimestamp|null);

            /** Vector for the head to look towards (camera to point in torso space) */
            uPCt?: (Ivec3|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            servos?: ({ [k: string]: message.actuation.IServoState }|null);
        }

        /** Head motion information including look direction, time and gain and torque for each servo */
        class HeadIK implements IHeadIK {

            /**
             * Constructs a new HeadIK.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IHeadIK);

            /** When the command should execute */
            public time?: (google.protobuf.ITimestamp|null);

            /** Vector for the head to look towards (camera to point in torso space) */
            public uPCt?: (Ivec3|null);

            /** Gain and torque of each servo, with the key corresponding to ServoID */
            public servos: { [k: string]: message.actuation.IServoState };

            /**
             * Encodes the specified HeadIK message. Does not implicitly {@link message.actuation.HeadIK.verify|verify} messages.
             * @param message HeadIK message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IHeadIK, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a HeadIK message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns HeadIK
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.HeadIK;
        }

        /** Properties of a ServoState. */
        interface IServoState {

            /** values are in range 0-100. */
            gain?: (number|null);

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            torque?: (number|null);
        }

        /** Represents a ServoState. */
        class ServoState implements IServoState {

            /**
             * Constructs a new ServoState.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IServoState);

            /** values are in range 0-100. */
            public gain: number;

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            public torque: number;

            /**
             * Encodes the specified ServoState message. Does not implicitly {@link message.actuation.ServoState.verify|verify} messages.
             * @param message ServoState message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IServoState, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoState message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoState
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.ServoState;
        }

        /** Properties of a ServoCommand. */
        interface IServoCommand {

            /** The time that the servo should reach the given position */
            time?: (google.protobuf.ITimestamp|null);

            /** The position value of the destination in radians. */
            position?: (number|null);

            /** Gain and torque of the servo */
            state?: (message.actuation.IServoState|null);
        }

        /** Represents a ServoCommand. */
        class ServoCommand implements IServoCommand {

            /**
             * Constructs a new ServoCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IServoCommand);

            /** The time that the servo should reach the given position */
            public time?: (google.protobuf.ITimestamp|null);

            /** The position value of the destination in radians. */
            public position: number;

            /** Gain and torque of the servo */
            public state?: (message.actuation.IServoState|null);

            /**
             * Encodes the specified ServoCommand message. Does not implicitly {@link message.actuation.ServoCommand.verify|verify} messages.
             * @param message ServoCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IServoCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.ServoCommand;
        }

        /** Properties of a RightShoulderPitch. */
        interface IRightShoulderPitch {

            /** RightShoulderPitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightShoulderPitch. */
        class RightShoulderPitch implements IRightShoulderPitch {

            /**
             * Constructs a new RightShoulderPitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightShoulderPitch);

            /** RightShoulderPitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightShoulderPitch message. Does not implicitly {@link message.actuation.RightShoulderPitch.verify|verify} messages.
             * @param message RightShoulderPitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightShoulderPitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightShoulderPitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightShoulderPitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightShoulderPitch;
        }

        /** Properties of a LeftShoulderPitch. */
        interface ILeftShoulderPitch {

            /** LeftShoulderPitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftShoulderPitch. */
        class LeftShoulderPitch implements ILeftShoulderPitch {

            /**
             * Constructs a new LeftShoulderPitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftShoulderPitch);

            /** LeftShoulderPitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftShoulderPitch message. Does not implicitly {@link message.actuation.LeftShoulderPitch.verify|verify} messages.
             * @param message LeftShoulderPitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftShoulderPitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftShoulderPitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftShoulderPitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftShoulderPitch;
        }

        /** Properties of a RightShoulderRoll. */
        interface IRightShoulderRoll {

            /** RightShoulderRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightShoulderRoll. */
        class RightShoulderRoll implements IRightShoulderRoll {

            /**
             * Constructs a new RightShoulderRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightShoulderRoll);

            /** RightShoulderRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightShoulderRoll message. Does not implicitly {@link message.actuation.RightShoulderRoll.verify|verify} messages.
             * @param message RightShoulderRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightShoulderRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightShoulderRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightShoulderRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightShoulderRoll;
        }

        /** Properties of a LeftShoulderRoll. */
        interface ILeftShoulderRoll {

            /** LeftShoulderRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftShoulderRoll. */
        class LeftShoulderRoll implements ILeftShoulderRoll {

            /**
             * Constructs a new LeftShoulderRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftShoulderRoll);

            /** LeftShoulderRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftShoulderRoll message. Does not implicitly {@link message.actuation.LeftShoulderRoll.verify|verify} messages.
             * @param message LeftShoulderRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftShoulderRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftShoulderRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftShoulderRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftShoulderRoll;
        }

        /** Properties of a RightElbow. */
        interface IRightElbow {

            /** RightElbow command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightElbow. */
        class RightElbow implements IRightElbow {

            /**
             * Constructs a new RightElbow.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightElbow);

            /** RightElbow command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightElbow message. Does not implicitly {@link message.actuation.RightElbow.verify|verify} messages.
             * @param message RightElbow message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightElbow, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightElbow message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightElbow
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightElbow;
        }

        /** Properties of a LeftElbow. */
        interface ILeftElbow {

            /** LeftElbow command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftElbow. */
        class LeftElbow implements ILeftElbow {

            /**
             * Constructs a new LeftElbow.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftElbow);

            /** LeftElbow command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftElbow message. Does not implicitly {@link message.actuation.LeftElbow.verify|verify} messages.
             * @param message LeftElbow message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftElbow, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftElbow message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftElbow
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftElbow;
        }

        /** Properties of a RightHipYaw. */
        interface IRightHipYaw {

            /** RightHipYaw command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightHipYaw. */
        class RightHipYaw implements IRightHipYaw {

            /**
             * Constructs a new RightHipYaw.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightHipYaw);

            /** RightHipYaw command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightHipYaw message. Does not implicitly {@link message.actuation.RightHipYaw.verify|verify} messages.
             * @param message RightHipYaw message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightHipYaw, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightHipYaw message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightHipYaw
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightHipYaw;
        }

        /** Properties of a LeftHipYaw. */
        interface ILeftHipYaw {

            /** LeftHipYaw command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftHipYaw. */
        class LeftHipYaw implements ILeftHipYaw {

            /**
             * Constructs a new LeftHipYaw.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftHipYaw);

            /** LeftHipYaw command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftHipYaw message. Does not implicitly {@link message.actuation.LeftHipYaw.verify|verify} messages.
             * @param message LeftHipYaw message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftHipYaw, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftHipYaw message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftHipYaw
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftHipYaw;
        }

        /** Properties of a RightHipRoll. */
        interface IRightHipRoll {

            /** RightHipRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightHipRoll. */
        class RightHipRoll implements IRightHipRoll {

            /**
             * Constructs a new RightHipRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightHipRoll);

            /** RightHipRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightHipRoll message. Does not implicitly {@link message.actuation.RightHipRoll.verify|verify} messages.
             * @param message RightHipRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightHipRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightHipRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightHipRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightHipRoll;
        }

        /** Properties of a LeftHipRoll. */
        interface ILeftHipRoll {

            /** LeftHipRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftHipRoll. */
        class LeftHipRoll implements ILeftHipRoll {

            /**
             * Constructs a new LeftHipRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftHipRoll);

            /** LeftHipRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftHipRoll message. Does not implicitly {@link message.actuation.LeftHipRoll.verify|verify} messages.
             * @param message LeftHipRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftHipRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftHipRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftHipRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftHipRoll;
        }

        /** Properties of a RightHipPitch. */
        interface IRightHipPitch {

            /** RightHipPitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightHipPitch. */
        class RightHipPitch implements IRightHipPitch {

            /**
             * Constructs a new RightHipPitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightHipPitch);

            /** RightHipPitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightHipPitch message. Does not implicitly {@link message.actuation.RightHipPitch.verify|verify} messages.
             * @param message RightHipPitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightHipPitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightHipPitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightHipPitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightHipPitch;
        }

        /** Properties of a LeftHipPitch. */
        interface ILeftHipPitch {

            /** LeftHipPitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftHipPitch. */
        class LeftHipPitch implements ILeftHipPitch {

            /**
             * Constructs a new LeftHipPitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftHipPitch);

            /** LeftHipPitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftHipPitch message. Does not implicitly {@link message.actuation.LeftHipPitch.verify|verify} messages.
             * @param message LeftHipPitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftHipPitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftHipPitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftHipPitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftHipPitch;
        }

        /** Properties of a RightKnee. */
        interface IRightKnee {

            /** RightKnee command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightKnee. */
        class RightKnee implements IRightKnee {

            /**
             * Constructs a new RightKnee.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightKnee);

            /** RightKnee command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightKnee message. Does not implicitly {@link message.actuation.RightKnee.verify|verify} messages.
             * @param message RightKnee message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightKnee, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightKnee message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightKnee
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightKnee;
        }

        /** Properties of a LeftKnee. */
        interface ILeftKnee {

            /** LeftKnee command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftKnee. */
        class LeftKnee implements ILeftKnee {

            /**
             * Constructs a new LeftKnee.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftKnee);

            /** LeftKnee command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftKnee message. Does not implicitly {@link message.actuation.LeftKnee.verify|verify} messages.
             * @param message LeftKnee message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftKnee, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftKnee message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftKnee
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftKnee;
        }

        /** Properties of a RightAnklePitch. */
        interface IRightAnklePitch {

            /** RightAnklePitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightAnklePitch. */
        class RightAnklePitch implements IRightAnklePitch {

            /**
             * Constructs a new RightAnklePitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightAnklePitch);

            /** RightAnklePitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightAnklePitch message. Does not implicitly {@link message.actuation.RightAnklePitch.verify|verify} messages.
             * @param message RightAnklePitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightAnklePitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightAnklePitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightAnklePitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightAnklePitch;
        }

        /** Properties of a LeftAnklePitch. */
        interface ILeftAnklePitch {

            /** LeftAnklePitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftAnklePitch. */
        class LeftAnklePitch implements ILeftAnklePitch {

            /**
             * Constructs a new LeftAnklePitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftAnklePitch);

            /** LeftAnklePitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftAnklePitch message. Does not implicitly {@link message.actuation.LeftAnklePitch.verify|verify} messages.
             * @param message LeftAnklePitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftAnklePitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftAnklePitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftAnklePitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftAnklePitch;
        }

        /** Properties of a RightAnkleRoll. */
        interface IRightAnkleRoll {

            /** RightAnkleRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a RightAnkleRoll. */
        class RightAnkleRoll implements IRightAnkleRoll {

            /**
             * Constructs a new RightAnkleRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IRightAnkleRoll);

            /** RightAnkleRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified RightAnkleRoll message. Does not implicitly {@link message.actuation.RightAnkleRoll.verify|verify} messages.
             * @param message RightAnkleRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IRightAnkleRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RightAnkleRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RightAnkleRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.RightAnkleRoll;
        }

        /** Properties of a LeftAnkleRoll. */
        interface ILeftAnkleRoll {

            /** LeftAnkleRoll command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a LeftAnkleRoll. */
        class LeftAnkleRoll implements ILeftAnkleRoll {

            /**
             * Constructs a new LeftAnkleRoll.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.ILeftAnkleRoll);

            /** LeftAnkleRoll command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified LeftAnkleRoll message. Does not implicitly {@link message.actuation.LeftAnkleRoll.verify|verify} messages.
             * @param message LeftAnkleRoll message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.ILeftAnkleRoll, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LeftAnkleRoll message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LeftAnkleRoll
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.LeftAnkleRoll;
        }

        /** Properties of a HeadYaw. */
        interface IHeadYaw {

            /** HeadYaw command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a HeadYaw. */
        class HeadYaw implements IHeadYaw {

            /**
             * Constructs a new HeadYaw.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IHeadYaw);

            /** HeadYaw command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified HeadYaw message. Does not implicitly {@link message.actuation.HeadYaw.verify|verify} messages.
             * @param message HeadYaw message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IHeadYaw, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a HeadYaw message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns HeadYaw
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.HeadYaw;
        }

        /** Properties of a HeadPitch. */
        interface IHeadPitch {

            /** HeadPitch command */
            command?: (message.actuation.IServoCommand|null);
        }

        /** Represents a HeadPitch. */
        class HeadPitch implements IHeadPitch {

            /**
             * Constructs a new HeadPitch.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IHeadPitch);

            /** HeadPitch command. */
            public command?: (message.actuation.IServoCommand|null);

            /**
             * Encodes the specified HeadPitch message. Does not implicitly {@link message.actuation.HeadPitch.verify|verify} messages.
             * @param message HeadPitch message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IHeadPitch, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a HeadPitch message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns HeadPitch
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.HeadPitch;
        }

        /** Properties of a ServoTarget. */
        interface IServoTarget {

            /** When the target was created */
            time?: (google.protobuf.ITimestamp|null);

            /** The ID of the servo being controlled */
            id?: (number|null);

            /** triggered as set in Alarm LED/Shutdown */
            position?: (number|null);

            /** values are in range 0~254. K_p : p_gain / 8 */
            gain?: (number|null);

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            torque?: (number|null);
        }

        /** @author Trent Houliston */
        class ServoTarget implements IServoTarget {

            /**
             * Constructs a new ServoTarget.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IServoTarget);

            /** When the target was created */
            public time?: (google.protobuf.ITimestamp|null);

            /** The ID of the servo being controlled */
            public id: number;

            /** triggered as set in Alarm LED/Shutdown */
            public position: number;

            /** values are in range 0~254. K_p : p_gain / 8 */
            public gain: number;

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            public torque: number;

            /**
             * Encodes the specified ServoTarget message. Does not implicitly {@link message.actuation.ServoTarget.verify|verify} messages.
             * @param message ServoTarget message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IServoTarget, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoTarget message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoTarget
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.ServoTarget;
        }

        /** Properties of a ServoTargets. */
        interface IServoTargets {

            /** ServoTargets targets */
            targets?: (message.actuation.IServoTarget[]|null);
        }

        /** Represents a ServoTargets. */
        class ServoTargets implements IServoTargets {

            /**
             * Constructs a new ServoTargets.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.actuation.IServoTargets);

            /** ServoTargets targets. */
            public targets: message.actuation.IServoTarget[];

            /**
             * Encodes the specified ServoTargets message. Does not implicitly {@link message.actuation.ServoTargets.verify|verify} messages.
             * @param message ServoTargets message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.actuation.IServoTargets, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoTargets message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoTargets
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.actuation.ServoTargets;
        }
    }

    /** Namespace behaviour. */
    namespace behaviour {

        /** Properties of a Behaviour. */
        interface IBehaviour {

            /** Behaviour state */
            state?: (message.behaviour.Behaviour.State|null);
        }

        /** Represents a Behaviour. */
        class Behaviour implements IBehaviour {

            /**
             * Constructs a new Behaviour.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IBehaviour);

            /** Behaviour state. */
            public state: message.behaviour.Behaviour.State;

            /**
             * Encodes the specified Behaviour message. Does not implicitly {@link message.behaviour.Behaviour.verify|verify} messages.
             * @param message Behaviour message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IBehaviour, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Behaviour message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Behaviour
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.Behaviour;
        }

        namespace Behaviour {

            /** State enum. */
            enum State {
                UNKNOWN = 0,
                INIT = 1,
                SEARCH_FOR_BALL = 2,
                SEARCH_FOR_GOALS = 3,
                WALK_TO_BALL = 4,
                PICKED_UP = 5,
                INITIAL = 6,
                READY = 7,
                SET = 8,
                TIMEOUT = 9,
                FINISHED = 10,
                PENALISED = 11,
                GOALIE_WALK = 12,
                MOVE_TO_CENTRE = 13,
                LOCALISING = 14,
                SHOOTOUT = 15
            }
        }

        /** Properties of a KickPlan. */
        interface IKickPlan {

            /** Where the robot wants to kick to. In field space */
            target?: (Ivec2|null);

            /** KickPlan kickType */
            kickType?: (message.behaviour.KickPlan.KickType|null);
        }

        /** Represents a KickPlan. */
        class KickPlan implements IKickPlan {

            /**
             * Constructs a new KickPlan.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IKickPlan);

            /** Where the robot wants to kick to. In field space */
            public target?: (Ivec2|null);

            /** KickPlan kickType. */
            public kickType: message.behaviour.KickPlan.KickType;

            /**
             * Encodes the specified KickPlan message. Does not implicitly {@link message.behaviour.KickPlan.verify|verify} messages.
             * @param message KickPlan message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IKickPlan, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickPlan message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickPlan
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.KickPlan;
        }

        namespace KickPlan {

            /** KickType enum. */
            enum KickType {
                SCRIPTED = 0,
                IK_KICK = 1,
                PENALTY = 2
            }
        }

        /** Properties of a WantsToKick. */
        interface IWantsToKick {

            /** WantsToKick kick */
            kick?: (boolean|null);
        }

        /** Represents a WantsToKick. */
        class WantsToKick implements IWantsToKick {

            /**
             * Constructs a new WantsToKick.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IWantsToKick);

            /** WantsToKick kick. */
            public kick: boolean;

            /**
             * Encodes the specified WantsToKick message. Does not implicitly {@link message.behaviour.WantsToKick.verify|verify} messages.
             * @param message WantsToKick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IWantsToKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WantsToKick message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WantsToKick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.WantsToKick;
        }

        /** Properties of a MotionCommand. */
        interface IMotionCommand {

            /** The type of this motion command */
            type?: (message.behaviour.MotionCommand.Type|null);

            /** Required data for WalkToState command */
            goalState?: (Ivec3|null);

            /** Required data for WalkToBall command */
            kickTarget?: (Ivec2|null);

            /** Required data for DirectCommand command */
            walkCommand?: (Ivec3|null);

            /** Required data for RotateOnSpot command */
            clockwise?: (boolean|null);
        }

        /** Details of which walk behaviour to execute and where on the field to do it */
        class MotionCommand implements IMotionCommand {

            /**
             * Constructs a new MotionCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IMotionCommand);

            /** The type of this motion command */
            public type: message.behaviour.MotionCommand.Type;

            /** Required data for WalkToState command */
            public goalState?: (Ivec3|null);

            /** Required data for WalkToBall command */
            public kickTarget?: (Ivec2|null);

            /** Required data for DirectCommand command */
            public walkCommand?: (Ivec3|null);

            /** Required data for RotateOnSpot command */
            public clockwise: boolean;

            /**
             * Encodes the specified MotionCommand message. Does not implicitly {@link message.behaviour.MotionCommand.verify|verify} messages.
             * @param message MotionCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IMotionCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MotionCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MotionCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.MotionCommand;
        }

        namespace MotionCommand {

            /** Type enum. */
            enum Type {
                STAND_STILL = 0,
                WALK_TO_STATE = 1,
                BALL_APPROACH = 2,
                DIRECT_COMMAND = 3,
                ROTATE_ON_SPOT = 4,
                WALK_TO_READY = 5,
                ROTATE_AROUND_BALL = 6
            }
        }

        /** Properties of a Nod. */
        interface INod {

            /** Nod value */
            value?: (boolean|null);
        }

        /** Represents a Nod. */
        class Nod implements INod {

            /**
             * Constructs a new Nod.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.INod);

            /** Nod value. */
            public value: boolean;

            /**
             * Encodes the specified Nod message. Does not implicitly {@link message.behaviour.Nod.verify|verify} messages.
             * @param message Nod message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.INod, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Nod message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Nod
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.Nod;
        }

        /** Properties of a ServoCommand. */
        interface IServoCommand {

            /** The subsumption ID which created the command */
            source?: (number|Long|null);

            /** The time when the command was created */
            time?: (google.protobuf.ITimestamp|null);

            /** The servo which is being commanded */
            id?: (number|null);

            /** is triggered as set in Alarm LED/Shutdown */
            position?: (number|null);

            /** K_p : p_gain / 8 */
            gain?: (number|null);

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            torque?: (number|null);
        }

        /** Represents a ServoCommand. */
        class ServoCommand implements IServoCommand {

            /**
             * Constructs a new ServoCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IServoCommand);

            /** The subsumption ID which created the command */
            public source: (number|Long);

            /** The time when the command was created */
            public time?: (google.protobuf.ITimestamp|null);

            /** The servo which is being commanded */
            public id: number;

            /** is triggered as set in Alarm LED/Shutdown */
            public position: number;

            /** K_p : p_gain / 8 */
            public gain: number;

            /** Used to set the servo on or off. Typically either 0 (off) or 100 (on) */
            public torque: number;

            /**
             * Encodes the specified ServoCommand message. Does not implicitly {@link message.behaviour.ServoCommand.verify|verify} messages.
             * @param message ServoCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IServoCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.ServoCommand;
        }

        /** Properties of a ServoCommands. */
        interface IServoCommands {

            /** ServoCommands commands */
            commands?: (message.behaviour.IServoCommand[]|null);
        }

        /** Represents a ServoCommands. */
        class ServoCommands implements IServoCommands {

            /**
             * Constructs a new ServoCommands.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.behaviour.IServoCommands);

            /** ServoCommands commands. */
            public commands: message.behaviour.IServoCommand[];

            /**
             * Encodes the specified ServoCommands message. Does not implicitly {@link message.behaviour.ServoCommands.verify|verify} messages.
             * @param message ServoCommands message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.behaviour.IServoCommands, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServoCommands message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServoCommands
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.behaviour.ServoCommands;
        }

        /** Namespace state. */
        namespace state {

            /** Stability enum. */
            enum Stability {
                FALLEN = 0,
                FALLING = 1,
                DYNAMIC = 2,
                STATIC = 3,
                STANDING = 4
            }
        }
    }

    /** Properties of a Line. */
    interface ILine {

        /** Line normal */
        normal?: (Ifvec3|null);

        /** Line distance */
        distance?: (number|null);
    }

    /** Hesse normal form representation of a line. See https://en.wikipedia.org/wiki/Hesse_normal_form */
    class Line implements ILine {

        /**
         * Constructs a new Line.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.ILine);

        /** Line normal. */
        public normal?: (Ifvec3|null);

        /** Line distance. */
        public distance: number;

        /**
         * Encodes the specified Line message. Does not implicitly {@link message.Line.verify|verify} messages.
         * @param message Line message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.ILine, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Line message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Line
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Line;
    }

    /** Properties of a Circle. */
    interface ICircle {

        /** Circle radius */
        radius?: (number|null);

        /** Circle centre */
        centre?: (Ivec2|null);
    }

    /** Represents a Circle. */
    class Circle implements ICircle {

        /**
         * Constructs a new Circle.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.ICircle);

        /** Circle radius. */
        public radius: number;

        /** Circle centre. */
        public centre?: (Ivec2|null);

        /**
         * Encodes the specified Circle message. Does not implicitly {@link message.Circle.verify|verify} messages.
         * @param message Circle message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.ICircle, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Circle message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Circle
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Circle;
    }

    /** Properties of an Ellipse. */
    interface IEllipse {

        /** See http://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections */
        ellipse?: (Imat3|null);
    }

    /** A 3D matrix representation of the Ellipse */
    class Ellipse implements IEllipse {

        /**
         * Constructs a new Ellipse.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.IEllipse);

        /** See http://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections */
        public ellipse?: (Imat3|null);

        /**
         * Encodes the specified Ellipse message. Does not implicitly {@link message.Ellipse.verify|verify} messages.
         * @param message Ellipse message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.IEllipse, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes an Ellipse message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Ellipse
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Ellipse;
    }

    /** Properties of a Quad. */
    interface IQuad {

        /** Quad tl */
        tl?: (Ivec2|null);

        /** Quad tr */
        tr?: (Ivec2|null);

        /** Quad bl */
        bl?: (Ivec2|null);

        /** Quad br */
        br?: (Ivec2|null);
    }

    /** Four corner points of a quadrilateral */
    class Quad implements IQuad {

        /**
         * Constructs a new Quad.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.IQuad);

        /** Quad tl. */
        public tl?: (Ivec2|null);

        /** Quad tr. */
        public tr?: (Ivec2|null);

        /** Quad bl. */
        public bl?: (Ivec2|null);

        /** Quad br. */
        public br?: (Ivec2|null);

        /**
         * Encodes the specified Quad message. Does not implicitly {@link message.Quad.verify|verify} messages.
         * @param message Quad message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.IQuad, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Quad message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Quad
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Quad;
    }

    /** Properties of a Polygon. */
    interface IPolygon {

        /** Polygon vertices */
        vertices?: (Ivec2[]|null);
    }

    /** Represents a Polygon. */
    class Polygon implements IPolygon {

        /**
         * Constructs a new Polygon.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.IPolygon);

        /** Polygon vertices. */
        public vertices: Ivec2[];

        /**
         * Encodes the specified Polygon message. Does not implicitly {@link message.Polygon.verify|verify} messages.
         * @param message Polygon message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.IPolygon, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Polygon message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Polygon
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Polygon;
    }

    /** Properties of a Frustum. */
    interface IFrustum {

        /** Frustum tl */
        tl?: (Ivec3|null);

        /** Frustum tr */
        tr?: (Ivec3|null);

        /** Frustum bl */
        bl?: (Ivec3|null);

        /** Frustum br */
        br?: (Ivec3|null);
    }

    /** Contains the positions of the four corners of a frustrum */
    class Frustum implements IFrustum {

        /**
         * Constructs a new Frustum.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.IFrustum);

        /** Frustum tl. */
        public tl?: (Ivec3|null);

        /** Frustum tr. */
        public tr?: (Ivec3|null);

        /** Frustum bl. */
        public bl?: (Ivec3|null);

        /** Frustum br. */
        public br?: (Ivec3|null);

        /**
         * Encodes the specified Frustum message. Does not implicitly {@link message.Frustum.verify|verify} messages.
         * @param message Frustum message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.IFrustum, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Frustum message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Frustum
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Frustum;
    }

    /** Properties of a Cone. */
    interface ICone {

        /** 3D axis of the centre of the cone */
        axis?: (Ifvec3|null);

        /** Cos(theta) of the cone angle */
        radius?: (number|null);
    }

    /** Represents a Cone. */
    class Cone implements ICone {

        /**
         * Constructs a new Cone.
         * @param [properties] Properties to set
         */
        constructor(properties?: message.ICone);

        /** 3D axis of the centre of the cone */
        public axis?: (Ifvec3|null);

        /** Cos(theta) of the cone angle */
        public radius: number;

        /**
         * Encodes the specified Cone message. Does not implicitly {@link message.Cone.verify|verify} messages.
         * @param message Cone message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: message.ICone, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Cone message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Cone
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.Cone;
    }

    /** Namespace input. */
    namespace input {

        /** Properties of a GameEvents. */
        interface IGameEvents {
        }

        /** Represents a GameEvents. */
        class GameEvents implements IGameEvents {

            /**
             * Constructs a new GameEvents.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.input.IGameEvents);

            /**
             * Encodes the specified GameEvents message. Does not implicitly {@link message.input.GameEvents.verify|verify} messages.
             * @param message GameEvents message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.input.IGameEvents, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GameEvents message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GameEvents
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents;
        }

        namespace GameEvents {

            /** Context describes which actor/team is being referred to by an event */
            enum Context {
                UNKNOWN_CONTEXT = 0,
                SELF = 1,
                TEAM = 2,
                OPPONENT = 3,
                UNKNOWN = 4
            }

            /** TeamColour enum. */
            enum TeamColour {
                UNKNOWN_TEAM_COLOUR = 0,
                CYAN = 1,
                MAGENTA = 2
            }

            /** Properties of a Score. */
            interface IScore {

                /** Score ownScore */
                ownScore?: (number|null);

                /** Score opponentScore */
                opponentScore?: (number|null);
            }

            /** Represents a Score. */
            class Score implements IScore {

                /**
                 * Constructs a new Score.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IScore);

                /** Score ownScore. */
                public ownScore: number;

                /** Score opponentScore. */
                public opponentScore: number;

                /**
                 * Encodes the specified Score message. Does not implicitly {@link message.input.GameEvents.Score.verify|verify} messages.
                 * @param message Score message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IScore, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Score message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Score
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.Score;
            }

            /** Properties of a GoalScored. */
            interface IGoalScored {

                /** The actor/team who scored the goal */
                context?: (message.input.GameEvents.Context|null);

                /** GoalScored totalScore */
                totalScore?: (number|null);
            }

            /** Represents a GoalScored. */
            class GoalScored implements IGoalScored {

                /**
                 * Constructs a new GoalScored.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IGoalScored);

                /** The actor/team who scored the goal */
                public context: message.input.GameEvents.Context;

                /** GoalScored totalScore. */
                public totalScore: number;

                /**
                 * Encodes the specified GoalScored message. Does not implicitly {@link message.input.GameEvents.GoalScored.verify|verify} messages.
                 * @param message GoalScored message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IGoalScored, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a GoalScored message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns GoalScored
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.GoalScored;
            }

            /** Properties of a Penalisation. */
            interface IPenalisation {

                /** Penalisation context */
                context?: (message.input.GameEvents.Context|null);

                /** Penalisation robotId */
                robotId?: (number|null);

                /** The timestamp when the robot may resume playing */
                ends?: (google.protobuf.ITimestamp|null);

                /** Penalisation reason */
                reason?: (message.input.GameState.Data.PenaltyReason|null);
            }

            /** Represents a Penalisation. */
            class Penalisation implements IPenalisation {

                /**
                 * Constructs a new Penalisation.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IPenalisation);

                /** Penalisation context. */
                public context: message.input.GameEvents.Context;

                /** Penalisation robotId. */
                public robotId: number;

                /** The timestamp when the robot may resume playing */
                public ends?: (google.protobuf.ITimestamp|null);

                /** Penalisation reason. */
                public reason: message.input.GameState.Data.PenaltyReason;

                /**
                 * Encodes the specified Penalisation message. Does not implicitly {@link message.input.GameEvents.Penalisation.verify|verify} messages.
                 * @param message Penalisation message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IPenalisation, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Penalisation message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Penalisation
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.Penalisation;
            }

            /** Properties of an Unpenalisation. */
            interface IUnpenalisation {

                /** Unpenalisation context */
                context?: (message.input.GameEvents.Context|null);

                /** Unpenalisation robotId */
                robotId?: (number|null);
            }

            /** The context and robot which were unpenalised */
            class Unpenalisation implements IUnpenalisation {

                /**
                 * Constructs a new Unpenalisation.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IUnpenalisation);

                /** Unpenalisation context. */
                public context: message.input.GameEvents.Context;

                /** Unpenalisation robotId. */
                public robotId: number;

                /**
                 * Encodes the specified Unpenalisation message. Does not implicitly {@link message.input.GameEvents.Unpenalisation.verify|verify} messages.
                 * @param message Unpenalisation message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IUnpenalisation, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an Unpenalisation message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Unpenalisation
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.Unpenalisation;
            }

            /** Properties of a CoachMessage. */
            interface ICoachMessage {

                /** The intended recipient(s) */
                context?: (message.input.GameEvents.Context|null);

                /** CoachMessage message */
                message?: (string|null);
            }

            /** Represents a CoachMessage. */
            class CoachMessage implements ICoachMessage {

                /**
                 * Constructs a new CoachMessage.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.ICoachMessage);

                /** The intended recipient(s) */
                public context: message.input.GameEvents.Context;

                /** CoachMessage message. */
                public message: string;

                /**
                 * Encodes the specified CoachMessage message. Does not implicitly {@link message.input.GameEvents.CoachMessage.verify|verify} messages.
                 * @param message CoachMessage message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.ICoachMessage, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a CoachMessage message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns CoachMessage
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.CoachMessage;
            }

            /** Properties of a HalfTime. */
            interface IHalfTime {

                /** HalfTime firstHalf */
                firstHalf?: (boolean|null);
            }

            /** Represents a HalfTime. */
            class HalfTime implements IHalfTime {

                /**
                 * Constructs a new HalfTime.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IHalfTime);

                /** HalfTime firstHalf. */
                public firstHalf: boolean;

                /**
                 * Encodes the specified HalfTime message. Does not implicitly {@link message.input.GameEvents.HalfTime.verify|verify} messages.
                 * @param message HalfTime message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IHalfTime, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a HalfTime message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns HalfTime
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.HalfTime;
            }

            /** Properties of a BallKickedOut. */
            interface IBallKickedOut {

                /** The party which kicked the ball out */
                context?: (message.input.GameEvents.Context|null);

                /** BallKickedOut time */
                time?: (google.protobuf.ITimestamp|null);
            }

            /** Represents a BallKickedOut. */
            class BallKickedOut implements IBallKickedOut {

                /**
                 * Constructs a new BallKickedOut.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IBallKickedOut);

                /** The party which kicked the ball out */
                public context: message.input.GameEvents.Context;

                /** BallKickedOut time. */
                public time?: (google.protobuf.ITimestamp|null);

                /**
                 * Encodes the specified BallKickedOut message. Does not implicitly {@link message.input.GameEvents.BallKickedOut.verify|verify} messages.
                 * @param message BallKickedOut message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IBallKickedOut, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a BallKickedOut message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns BallKickedOut
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.BallKickedOut;
            }

            /** Properties of a KickOffTeam. */
            interface IKickOffTeam {

                /** KickOffTeam context */
                context?: (message.input.GameEvents.Context|null);
            }

            /** Represents a KickOffTeam. */
            class KickOffTeam implements IKickOffTeam {

                /**
                 * Constructs a new KickOffTeam.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IKickOffTeam);

                /** KickOffTeam context. */
                public context: message.input.GameEvents.Context;

                /**
                 * Encodes the specified KickOffTeam message. Does not implicitly {@link message.input.GameEvents.KickOffTeam.verify|verify} messages.
                 * @param message KickOffTeam message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IKickOffTeam, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a KickOffTeam message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns KickOffTeam
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.KickOffTeam;
            }

            /** Properties of a GamePhase. */
            interface IGamePhase {

                /** GamePhase phase */
                phase?: (message.input.GameState.Data.Phase|null);

                /** Set some time after game-states INITIAL or READY */
                readyTime?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-states SET or PLAYING */
                endHalf?: (google.protobuf.ITimestamp|null);

                /** GamePhase ballFree */
                ballFree?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-state TIMEOUT */
                ends?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-state FINISHED */
                nextHalf?: (google.protobuf.ITimestamp|null);
            }

            /** Represents a GamePhase. */
            class GamePhase implements IGamePhase {

                /**
                 * Constructs a new GamePhase.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IGamePhase);

                /** GamePhase phase. */
                public phase: message.input.GameState.Data.Phase;

                /** Set some time after game-states INITIAL or READY */
                public readyTime?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-states SET or PLAYING */
                public endHalf?: (google.protobuf.ITimestamp|null);

                /** GamePhase ballFree. */
                public ballFree?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-state TIMEOUT */
                public ends?: (google.protobuf.ITimestamp|null);

                /** Set some time after game-state FINISHED */
                public nextHalf?: (google.protobuf.ITimestamp|null);

                /**
                 * Encodes the specified GamePhase message. Does not implicitly {@link message.input.GameEvents.GamePhase.verify|verify} messages.
                 * @param message GamePhase message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IGamePhase, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a GamePhase message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns GamePhase
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.GamePhase;
            }

            /** Properties of a GameMode. */
            interface IGameMode {

                /** GameMode mode */
                mode?: (message.input.GameState.Data.Mode|null);
            }

            /** Represents a GameMode. */
            class GameMode implements IGameMode {

                /**
                 * Constructs a new GameMode.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameEvents.IGameMode);

                /** GameMode mode. */
                public mode: message.input.GameState.Data.Mode;

                /**
                 * Encodes the specified GameMode message. Does not implicitly {@link message.input.GameEvents.GameMode.verify|verify} messages.
                 * @param message GameMode message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameEvents.IGameMode, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a GameMode message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns GameMode
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameEvents.GameMode;
            }
        }

        /** Properties of a GameState. */
        interface IGameState {

            /** GameState data */
            data?: (message.input.GameState.IData|null);
        }

        /** Represents a GameState. */
        class GameState implements IGameState {

            /**
             * Constructs a new GameState.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.input.IGameState);

            /** GameState data. */
            public data?: (message.input.GameState.IData|null);

            /**
             * Encodes the specified GameState message. Does not implicitly {@link message.input.GameState.verify|verify} messages.
             * @param message GameState message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.input.IGameState, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GameState message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GameState
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameState;
        }

        namespace GameState {

            /** Properties of a Data. */
            interface IData {

                /** Data phase */
                phase?: (message.input.GameState.Data.Phase|null);

                /** Data mode */
                mode?: (message.input.GameState.Data.Mode|null);

                /** Data firstHalf */
                firstHalf?: (boolean|null);

                /** Data kickedOutByUs */
                kickedOutByUs?: (boolean|null);

                /** Data kickedOutTime */
                kickedOutTime?: (google.protobuf.ITimestamp|null);

                /** Data ourKickOff */
                ourKickOff?: (boolean|null);

                /** Timestamp when the robot thinks the half is due to end */
                primaryTime?: (google.protobuf.ITimestamp|null);

                /** A timestamp used for other times. E.g. how long in ready, or how long until penalty ends */
                secondaryTime?: (google.protobuf.ITimestamp|null);

                /** Data team */
                team?: (message.input.GameState.Data.ITeam|null);

                /** Data opponent */
                opponent?: (message.input.GameState.Data.ITeam|null);

                /** Data self */
                self?: (message.input.GameState.Data.IRobot|null);
            }

            /** Represents a Data. */
            class Data implements IData {

                /**
                 * Constructs a new Data.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.GameState.IData);

                /** Data phase. */
                public phase: message.input.GameState.Data.Phase;

                /** Data mode. */
                public mode: message.input.GameState.Data.Mode;

                /** Data firstHalf. */
                public firstHalf: boolean;

                /** Data kickedOutByUs. */
                public kickedOutByUs: boolean;

                /** Data kickedOutTime. */
                public kickedOutTime?: (google.protobuf.ITimestamp|null);

                /** Data ourKickOff. */
                public ourKickOff: boolean;

                /** Timestamp when the robot thinks the half is due to end */
                public primaryTime?: (google.protobuf.ITimestamp|null);

                /** A timestamp used for other times. E.g. how long in ready, or how long until penalty ends */
                public secondaryTime?: (google.protobuf.ITimestamp|null);

                /** Data team. */
                public team?: (message.input.GameState.Data.ITeam|null);

                /** Data opponent. */
                public opponent?: (message.input.GameState.Data.ITeam|null);

                /** Data self. */
                public self?: (message.input.GameState.Data.IRobot|null);

                /**
                 * Encodes the specified Data message. Does not implicitly {@link message.input.GameState.Data.verify|verify} messages.
                 * @param message Data message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.GameState.IData, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Data message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Data
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameState.Data;
            }

            namespace Data {

                /** Mode enum. */
                enum Mode {
                    UNKNOWN_MODE = 0,
                    NORMAL = 1,
                    PENALTY_SHOOTOUT = 2,
                    OVERTIME = 3
                }

                /** Phase enum. */
                enum Phase {
                    UNKNOWN_PHASE = 0,
                    INITIAL = 1,
                    READY = 2,
                    SET = 3,
                    PLAYING = 4,
                    TIMEOUT = 5,
                    FINISHED = 6
                }

                /** PenaltyReason enum. */
                enum PenaltyReason {
                    UNKNOWN_PENALTY_REASON = 0,
                    UNPENALISED = 1,
                    BALL_MANIPULATION = 2,
                    PHYSICAL_CONTACT = 3,
                    ILLEGAL_ATTACK = 4,
                    ILLEGAL_DEFENSE = 5,
                    REQUEST_FOR_PICKUP = 6,
                    REQUEST_FOR_SERVICE = 7,
                    REQUEST_FOR_PICKUP_TO_SERVICE = 8,
                    SUBSTITUTE = 9,
                    MANUAL = 10,
                    PLAYER_PUSHING = 11
                }

                /** Properties of a Robot. */
                interface IRobot {

                    /** Robot id */
                    id?: (number|null);

                    /** Robot penaltyReason */
                    penaltyReason?: (message.input.GameState.Data.PenaltyReason|null);

                    /** The timestamp when the robot will be playing again */
                    unpenalised?: (google.protobuf.ITimestamp|null);
                }

                /** Used to keep track of penalised robots */
                class Robot implements IRobot {

                    /**
                     * Constructs a new Robot.
                     * @param [properties] Properties to set
                     */
                    constructor(properties?: message.input.GameState.Data.IRobot);

                    /** Robot id. */
                    public id: number;

                    /** Robot penaltyReason. */
                    public penaltyReason: message.input.GameState.Data.PenaltyReason;

                    /** The timestamp when the robot will be playing again */
                    public unpenalised?: (google.protobuf.ITimestamp|null);

                    /**
                     * Encodes the specified Robot message. Does not implicitly {@link message.input.GameState.Data.Robot.verify|verify} messages.
                     * @param message Robot message or plain object to encode
                     * @param [writer] Writer to encode to
                     * @returns Writer
                     */
                    public static encode(message: message.input.GameState.Data.IRobot, writer?: $protobuf.Writer): $protobuf.Writer;

                    /**
                     * Decodes a Robot message from the specified reader or buffer.
                     * @param reader Reader or buffer to decode from
                     * @param [length] Message length if known beforehand
                     * @returns Robot
                     * @throws {Error} If the payload is not a reader or valid buffer
                     * @throws {$protobuf.util.ProtocolError} If required fields are missing
                     */
                    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameState.Data.Robot;
                }

                /** Properties of a Team. */
                interface ITeam {

                    /** Team teamId */
                    teamId?: (number|null);

                    /** Team score */
                    score?: (number|null);

                    /** Team coachMessage */
                    coachMessage?: (string|null);

                    /** Team players */
                    players?: (message.input.GameState.Data.IRobot[]|null);
                }

                /** Represents a Team. */
                class Team implements ITeam {

                    /**
                     * Constructs a new Team.
                     * @param [properties] Properties to set
                     */
                    constructor(properties?: message.input.GameState.Data.ITeam);

                    /** Team teamId. */
                    public teamId: number;

                    /** Team score. */
                    public score: number;

                    /** Team coachMessage. */
                    public coachMessage: string;

                    /** Team players. */
                    public players: message.input.GameState.Data.IRobot[];

                    /**
                     * Encodes the specified Team message. Does not implicitly {@link message.input.GameState.Data.Team.verify|verify} messages.
                     * @param message Team message or plain object to encode
                     * @param [writer] Writer to encode to
                     * @returns Writer
                     */
                    public static encode(message: message.input.GameState.Data.ITeam, writer?: $protobuf.Writer): $protobuf.Writer;

                    /**
                     * Decodes a Team message from the specified reader or buffer.
                     * @param reader Reader or buffer to decode from
                     * @param [length] Message length if known beforehand
                     * @returns Team
                     * @throws {Error} If the payload is not a reader or valid buffer
                     * @throws {$protobuf.util.ProtocolError} If required fields are missing
                     */
                    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.GameState.Data.Team;
                }
            }
        }

        /** Properties of an Image. */
        interface IImage {

            /** see module::output::compressor::turbojpeg::Compressor::compress for currently supported formats */
            format?: (number|null);

            /** Number of pixels of {x, y} of the image */
            dimensions?: (Iuvec2|null);

            /** Image data */
            data?: (Uint8Array|null);

            /** Image id */
            id?: (number|null);

            /** Image name */
            name?: (string|null);

            /** Image timestamp */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** World to camera Isometry3d transform */
            Hcw?: (Imat4|null);

            /** Image lens */
            lens?: (message.input.Image.ILens|null);

            /** Ground truth data from webots */
            visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);
        }

        /** Represents an Image. */
        class Image implements IImage {

            /**
             * Constructs a new Image.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.input.IImage);

            /** see module::output::compressor::turbojpeg::Compressor::compress for currently supported formats */
            public format: number;

            /** Number of pixels of {x, y} of the image */
            public dimensions?: (Iuvec2|null);

            /** Image data. */
            public data: Uint8Array;

            /** Image id. */
            public id: number;

            /** Image name. */
            public name: string;

            /** Image timestamp. */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** World to camera Isometry3d transform */
            public Hcw?: (Imat4|null);

            /** Image lens. */
            public lens?: (message.input.Image.ILens|null);

            /** Ground truth data from webots */
            public visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);

            /**
             * Encodes the specified Image message. Does not implicitly {@link message.input.Image.verify|verify} messages.
             * @param message Image message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.input.IImage, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an Image message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Image
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Image;
        }

        namespace Image {

            /** Properties of a Lens. */
            interface ILens {

                /** Lens projection */
                projection?: (message.input.Image.Lens.Projection|null);

                /** Normalised focal length: focal length in pixels / image width */
                focalLength?: (number|null);

                /** The angular diameter that the lens covers (the area that light hits on the sensor). In radians */
                fov?: (number|null);

                /** Normalised image centre offset: pixels from centre to optical axis / image width */
                centre?: (Ifvec2|null);

                /** Lens k */
                k?: (Ifvec2|null);
            }

            /** Represents a Lens. */
            class Lens implements ILens {

                /**
                 * Constructs a new Lens.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.Image.ILens);

                /** Lens projection. */
                public projection: message.input.Image.Lens.Projection;

                /** Normalised focal length: focal length in pixels / image width */
                public focalLength: number;

                /** The angular diameter that the lens covers (the area that light hits on the sensor). In radians */
                public fov: number;

                /** Normalised image centre offset: pixels from centre to optical axis / image width */
                public centre?: (Ifvec2|null);

                /** Lens k. */
                public k?: (Ifvec2|null);

                /**
                 * Encodes the specified Lens message. Does not implicitly {@link message.input.Image.Lens.verify|verify} messages.
                 * @param message Lens message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.Image.ILens, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Lens message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Lens
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Image.Lens;
            }

            namespace Lens {

                /** Projection enum. */
                enum Projection {
                    UNKNOWN = 0,
                    RECTILINEAR = 1,
                    EQUIDISTANT = 2,
                    EQUISOLID = 3
                }
            }
        }

        /** Properties of a MotionCapture. */
        interface IMotionCapture {

            /** MotionCapture frameNumber */
            frameNumber?: (number|null);

            /** MotionCapture latency */
            latency?: (number|null);

            /** https://en.wikipedia.org/wiki/SMPTE_timecode */
            timecode?: (number|null);

            /** MotionCapture timecodeSub */
            timecodeSub?: (number|null);

            /** MotionCapture natnetTimestamp */
            natnetTimestamp?: (number|null);

            /** MotionCapture midExposureTimestamp */
            midExposureTimestamp?: (number|null);

            /** MotionCapture dataReceivedTimestamp */
            dataReceivedTimestamp?: (number|null);

            /** MotionCapture transmitTimestamp */
            transmitTimestamp?: (number|null);

            /** Whether or not this frame is to be recorded */
            recording?: (boolean|null);

            /** MotionCapture trackedModelsChanged */
            trackedModelsChanged?: (boolean|null);

            /** MotionCapture markerSets */
            markerSets?: (message.input.MotionCapture.IMarkerSet[]|null);

            /** MotionCapture markers */
            markers?: (message.input.MotionCapture.IMarker[]|null);

            /** MotionCapture rigidBodies */
            rigidBodies?: (message.input.MotionCapture.IRigidBody[]|null);

            /** MotionCapture skeletons */
            skeletons?: (message.input.MotionCapture.ISkeleton[]|null);

            /** MotionCapture labeledMarkers */
            labeledMarkers?: (message.input.MotionCapture.ILabeledMarker[]|null);

            /** MotionCapture forcePlates */
            forcePlates?: (message.input.MotionCapture.IForcePlate[]|null);

            /** MotionCapture devices */
            devices?: (message.input.MotionCapture.IDevice[]|null);
        }

        /**
         * Contains Marker, Rigidbody, and message metadata information.
         * We use optitrack for our mocap system. Their documentation is here:
         * https://v22.wiki.optitrack.com/index.php?title=OptiTrack_Documentation_Wiki
         */
        class MotionCapture implements IMotionCapture {

            /**
             * Constructs a new MotionCapture.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.input.IMotionCapture);

            /** MotionCapture frameNumber. */
            public frameNumber: number;

            /** MotionCapture latency. */
            public latency: number;

            /** https://en.wikipedia.org/wiki/SMPTE_timecode */
            public timecode: number;

            /** MotionCapture timecodeSub. */
            public timecodeSub: number;

            /** MotionCapture natnetTimestamp. */
            public natnetTimestamp: number;

            /** MotionCapture midExposureTimestamp. */
            public midExposureTimestamp: number;

            /** MotionCapture dataReceivedTimestamp. */
            public dataReceivedTimestamp: number;

            /** MotionCapture transmitTimestamp. */
            public transmitTimestamp: number;

            /** Whether or not this frame is to be recorded */
            public recording: boolean;

            /** MotionCapture trackedModelsChanged. */
            public trackedModelsChanged: boolean;

            /** MotionCapture markerSets. */
            public markerSets: message.input.MotionCapture.IMarkerSet[];

            /** MotionCapture markers. */
            public markers: message.input.MotionCapture.IMarker[];

            /** MotionCapture rigidBodies. */
            public rigidBodies: message.input.MotionCapture.IRigidBody[];

            /** MotionCapture skeletons. */
            public skeletons: message.input.MotionCapture.ISkeleton[];

            /** MotionCapture labeledMarkers. */
            public labeledMarkers: message.input.MotionCapture.ILabeledMarker[];

            /** MotionCapture forcePlates. */
            public forcePlates: message.input.MotionCapture.IForcePlate[];

            /** MotionCapture devices. */
            public devices: message.input.MotionCapture.IDevice[];

            /**
             * Encodes the specified MotionCapture message. Does not implicitly {@link message.input.MotionCapture.verify|verify} messages.
             * @param message MotionCapture message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.input.IMotionCapture, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MotionCapture message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MotionCapture
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture;
        }

        namespace MotionCapture {

            /** Properties of a Marker. */
            interface IMarker {

                /** Marker id */
                id?: (number|null);

                /** Marker position */
                position?: (Ifvec3|null);

                /** Marker size */
                size?: (number|null);
            }

            /** Mocap reference points to track */
            class Marker implements IMarker {

                /**
                 * Constructs a new Marker.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IMarker);

                /** Marker id. */
                public id: number;

                /** Marker position. */
                public position?: (Ifvec3|null);

                /** Marker size. */
                public size: number;

                /**
                 * Encodes the specified Marker message. Does not implicitly {@link message.input.MotionCapture.Marker.verify|verify} messages.
                 * @param message Marker message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IMarker, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Marker message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Marker
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.Marker;
            }

            /** Properties of a MarkerSet. */
            interface IMarkerSet {

                /** MarkerSet name */
                name?: (string|null);

                /** MarkerSet markers */
                markers?: (message.input.MotionCapture.IMarker[]|null);
            }

            /** Represents a MarkerSet. */
            class MarkerSet implements IMarkerSet {

                /**
                 * Constructs a new MarkerSet.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IMarkerSet);

                /** MarkerSet name. */
                public name: string;

                /** MarkerSet markers. */
                public markers: message.input.MotionCapture.IMarker[];

                /**
                 * Encodes the specified MarkerSet message. Does not implicitly {@link message.input.MotionCapture.MarkerSet.verify|verify} messages.
                 * @param message MarkerSet message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IMarkerSet, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MarkerSet message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MarkerSet
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.MarkerSet;
            }

            /** Properties of a RigidBody. */
            interface IRigidBody {

                /** RigidBody id */
                id?: (number|null);

                /** RigidBody position */
                position?: (Ifvec3|null);

                /** Quaternion representation of the rigid body's rotation */
                rotation?: (Ifvec4|null);

                /** RigidBody markers */
                markers?: (message.input.MotionCapture.IMarker[]|null);

                /** RigidBody error */
                error?: (number|null);

                /** RigidBody trackingValid */
                trackingValid?: (boolean|null);

                /** RigidBody name */
                name?: (string|null);

                /** RigidBody offset */
                offset?: (Ifvec3|null);

                /** RigidBody parent */
                parent?: (number|null);

                /** RigidBody children */
                children?: (number[]|null);
            }

            /** bones which make up skeletons are RigidBodys */
            class RigidBody implements IRigidBody {

                /**
                 * Constructs a new RigidBody.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IRigidBody);

                /** RigidBody id. */
                public id: number;

                /** RigidBody position. */
                public position?: (Ifvec3|null);

                /** Quaternion representation of the rigid body's rotation */
                public rotation?: (Ifvec4|null);

                /** RigidBody markers. */
                public markers: message.input.MotionCapture.IMarker[];

                /** RigidBody error. */
                public error: number;

                /** RigidBody trackingValid. */
                public trackingValid: boolean;

                /** RigidBody name. */
                public name: string;

                /** RigidBody offset. */
                public offset?: (Ifvec3|null);

                /** RigidBody parent. */
                public parent: number;

                /** RigidBody children. */
                public children: number[];

                /**
                 * Encodes the specified RigidBody message. Does not implicitly {@link message.input.MotionCapture.RigidBody.verify|verify} messages.
                 * @param message RigidBody message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IRigidBody, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a RigidBody message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns RigidBody
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.RigidBody;
            }

            /** Properties of a Skeleton. */
            interface ISkeleton {

                /** Skeleton id */
                id?: (number|null);

                /** Skeleton bones */
                bones?: (message.input.MotionCapture.IRigidBody[]|null);

                /** Skeleton name */
                name?: (string|null);
            }

            /** The whole character for which the motion represents */
            class Skeleton implements ISkeleton {

                /**
                 * Constructs a new Skeleton.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.ISkeleton);

                /** Skeleton id. */
                public id: number;

                /** Skeleton bones. */
                public bones: message.input.MotionCapture.IRigidBody[];

                /** Skeleton name. */
                public name: string;

                /**
                 * Encodes the specified Skeleton message. Does not implicitly {@link message.input.MotionCapture.Skeleton.verify|verify} messages.
                 * @param message Skeleton message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.ISkeleton, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Skeleton message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Skeleton
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.Skeleton;
            }

            /** Properties of a LabeledMarker. */
            interface ILabeledMarker {

                /** LabeledMarker marker */
                marker?: (message.input.MotionCapture.IMarker|null);

                /** LabeledMarker occluded */
                occluded?: (boolean|null);

                /** LabeledMarker pointCloudSolved */
                pointCloudSolved?: (boolean|null);

                /** LabeledMarker modelSolved */
                modelSolved?: (boolean|null);

                /** LabeledMarker err */
                err?: (number|null);

                /** LabeledMarker hasModel */
                hasModel?: (boolean|null);

                /** LabeledMarker unlabeled */
                unlabeled?: (boolean|null);

                /** LabeledMarker activeMarker */
                activeMarker?: (boolean|null);
            }

            /** Represents a LabeledMarker. */
            class LabeledMarker implements ILabeledMarker {

                /**
                 * Constructs a new LabeledMarker.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.ILabeledMarker);

                /** LabeledMarker marker. */
                public marker?: (message.input.MotionCapture.IMarker|null);

                /** LabeledMarker occluded. */
                public occluded: boolean;

                /** LabeledMarker pointCloudSolved. */
                public pointCloudSolved: boolean;

                /** LabeledMarker modelSolved. */
                public modelSolved: boolean;

                /** LabeledMarker err. */
                public err: number;

                /** LabeledMarker hasModel. */
                public hasModel: boolean;

                /** LabeledMarker unlabeled. */
                public unlabeled: boolean;

                /** LabeledMarker activeMarker. */
                public activeMarker: boolean;

                /**
                 * Encodes the specified LabeledMarker message. Does not implicitly {@link message.input.MotionCapture.LabeledMarker.verify|verify} messages.
                 * @param message LabeledMarker message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.ILabeledMarker, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a LabeledMarker message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns LabeledMarker
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.LabeledMarker;
            }

            /** Properties of a Channel. */
            interface IChannel {

                /** The changes for this degree of freedom over a given timespan */
                channel?: (number[]|null);
            }

            /** (DOF) */
            class Channel implements IChannel {

                /**
                 * Constructs a new Channel.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IChannel);

                /** The changes for this degree of freedom over a given timespan */
                public channel: number[];

                /**
                 * Encodes the specified Channel message. Does not implicitly {@link message.input.MotionCapture.Channel.verify|verify} messages.
                 * @param message Channel message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IChannel, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Channel message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Channel
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.Channel;
            }

            /** Properties of a ForcePlate. */
            interface IForcePlate {

                /** ForcePlate id */
                id?: (number|null);

                /** change over time */
                channels?: (message.input.MotionCapture.IChannel[]|null);
            }

            /** mocap footage */
            class ForcePlate implements IForcePlate {

                /**
                 * Constructs a new ForcePlate.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IForcePlate);

                /** ForcePlate id. */
                public id: number;

                /** change over time */
                public channels: message.input.MotionCapture.IChannel[];

                /**
                 * Encodes the specified ForcePlate message. Does not implicitly {@link message.input.MotionCapture.ForcePlate.verify|verify} messages.
                 * @param message ForcePlate message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IForcePlate, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a ForcePlate message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ForcePlate
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.ForcePlate;
            }

            /** Properties of a Device. */
            interface IDevice {

                /** Device id */
                id?: (number|null);

                /** Device channels */
                channels?: (message.input.MotionCapture.IChannel[]|null);
            }

            /** Represents a Device. */
            class Device implements IDevice {

                /**
                 * Constructs a new Device.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.MotionCapture.IDevice);

                /** Device id. */
                public id: number;

                /** Device channels. */
                public channels: message.input.MotionCapture.IChannel[];

                /**
                 * Encodes the specified Device message. Does not implicitly {@link message.input.MotionCapture.Device.verify|verify} messages.
                 * @param message Device message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.MotionCapture.IDevice, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Device message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Device
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.MotionCapture.Device;
            }
        }

        /** Properties of a Sensors. */
        interface ISensors {

            /** When the data were taken/captured */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** => 512 = 0g. Is {x, y, z} acceleration */
            accelerometer?: (Ivec3|null);

            /** => 512 = 0 degrees per second. Is {x, y, z} angular velocity */
            gyroscope?: (Ivec3|null);

            /** It measures the world in robot space */
            Htw?: (Imat4|null);

            /** Set of force sensitive resistors. Used to sense if a foot is on the ground */
            feet?: (message.input.Sensors.IFoot[]|null);

            /** Sensors servo */
            servo?: (message.input.Sensors.IServo[]|null);

            /** Sensors button */
            button?: (message.input.Sensors.IButton[]|null);

            /** Sensors led */
            led?: (message.input.Sensors.ILED[]|null);

            /** Voltage for the whole robot, from the main battery */
            voltage?: (number|null);

            /** Sensors battery */
            battery?: (number|null);

            /** A set of transforms; one for each servo */
            Htx?: (Imat4[]|null);

            /** Torso to Centre of Mass (CoM) in torso space */
            rMTt?: (Ivec4|null);

            /** Sensors Hgt */
            Hgt?: (Imat4|null);

            /** particle */
            inertiaTensor?: (Imat3|null);
        }

        /** Represents a Sensors. */
        class Sensors implements ISensors {

            /**
             * Constructs a new Sensors.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.input.ISensors);

            /** When the data were taken/captured */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** => 512 = 0g. Is {x, y, z} acceleration */
            public accelerometer?: (Ivec3|null);

            /** => 512 = 0 degrees per second. Is {x, y, z} angular velocity */
            public gyroscope?: (Ivec3|null);

            /** It measures the world in robot space */
            public Htw?: (Imat4|null);

            /** Set of force sensitive resistors. Used to sense if a foot is on the ground */
            public feet: message.input.Sensors.IFoot[];

            /** Sensors servo. */
            public servo: message.input.Sensors.IServo[];

            /** Sensors button. */
            public button: message.input.Sensors.IButton[];

            /** Sensors led. */
            public led: message.input.Sensors.ILED[];

            /** Voltage for the whole robot, from the main battery */
            public voltage: number;

            /** Sensors battery. */
            public battery: number;

            /** A set of transforms; one for each servo */
            public Htx: Imat4[];

            /** Torso to Centre of Mass (CoM) in torso space */
            public rMTt?: (Ivec4|null);

            /** Sensors Hgt. */
            public Hgt?: (Imat4|null);

            /** particle */
            public inertiaTensor?: (Imat3|null);

            /**
             * Encodes the specified Sensors message. Does not implicitly {@link message.input.Sensors.verify|verify} messages.
             * @param message Sensors message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.input.ISensors, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Sensors message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Sensors
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Sensors;
        }

        namespace Sensors {

            /** Properties of a Servo. */
            interface IServo {

                /** https://emanual.robotis.com/docs/en/dxl/mx/mx-106/#alarm-led17-shutdown18 */
                errorFlags?: (number|null);

                /** Servo id */
                id?: (number|null);

                /** Whether the torque is on or off */
                enabled?: (boolean|null);

                /** values are in range 0~254. K_p : p_gain / 8 */
                pGain?: (number|null);

                /** K_i : i_gain * 1000 / 2048 */
                iGain?: (number|null);

                /** derivatives. Gain values are in range 0~254. K_d : d_gain * 4 / 1000 */
                dGain?: (number|null);

                /** LED/Shutdown */
                goalPosition?: (number|null);

                /** Servo goalVelocity */
                goalVelocity?: (number|null);

                /** The range of the value is 0~4095 (0xFFF), and the unit is 0.088 [°] */
                presentPosition?: (number|null);

                /** direction */
                presentVelocity?: (number|null);

                /** maximum torque */
                load?: (number|null);

                /** set the address 17 and set 1 to the Bit 0 of the address 18 */
                voltage?: (number|null);

                /** Data value is identical to the actual temperature in Celsius */
                temperature?: (number|null);
            }

            /** https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ */
            class Servo implements IServo {

                /**
                 * Constructs a new Servo.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.Sensors.IServo);

                /** https://emanual.robotis.com/docs/en/dxl/mx/mx-106/#alarm-led17-shutdown18 */
                public errorFlags: number;

                /** Servo id. */
                public id: number;

                /** Whether the torque is on or off */
                public enabled: boolean;

                /** values are in range 0~254. K_p : p_gain / 8 */
                public pGain: number;

                /** K_i : i_gain * 1000 / 2048 */
                public iGain: number;

                /** derivatives. Gain values are in range 0~254. K_d : d_gain * 4 / 1000 */
                public dGain: number;

                /** LED/Shutdown */
                public goalPosition: number;

                /** Servo goalVelocity. */
                public goalVelocity: number;

                /** The range of the value is 0~4095 (0xFFF), and the unit is 0.088 [°] */
                public presentPosition: number;

                /** direction */
                public presentVelocity: number;

                /** maximum torque */
                public load: number;

                /** set the address 17 and set 1 to the Bit 0 of the address 18 */
                public voltage: number;

                /** Data value is identical to the actual temperature in Celsius */
                public temperature: number;

                /**
                 * Encodes the specified Servo message. Does not implicitly {@link message.input.Sensors.Servo.verify|verify} messages.
                 * @param message Servo message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.Sensors.IServo, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Servo message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Servo
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Sensors.Servo;
            }

            /** Properties of a Button. */
            interface IButton {

                /** Button id */
                id?: (number|null);

                /** True is pressed, false is unpressed */
                value?: (boolean|null);
            }

            /** Represents a Button. */
            class Button implements IButton {

                /**
                 * Constructs a new Button.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.Sensors.IButton);

                /** Button id. */
                public id: number;

                /** True is pressed, false is unpressed */
                public value: boolean;

                /**
                 * Encodes the specified Button message. Does not implicitly {@link message.input.Sensors.Button.verify|verify} messages.
                 * @param message Button message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.Sensors.IButton, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Button message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Button
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Sensors.Button;
            }

            /** Properties of a LED. */
            interface ILED {

                /** LED id */
                id?: (number|null);

                /** LED colour */
                colour?: (number|null);
            }

            /** Represents a LED. */
            class LED implements ILED {

                /**
                 * Constructs a new LED.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.Sensors.ILED);

                /** LED id. */
                public id: number;

                /** LED colour. */
                public colour: number;

                /**
                 * Encodes the specified LED message. Does not implicitly {@link message.input.Sensors.LED.verify|verify} messages.
                 * @param message LED message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.Sensors.ILED, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a LED message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns LED
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Sensors.LED;
            }

            /** Properties of a Foot. */
            interface IFoot {

                /** Foot down */
                down?: (boolean|null);

                /** Foot Hwf */
                Hwf?: (Imat4|null);
            }

            /** Represents a Foot. */
            class Foot implements IFoot {

                /**
                 * Constructs a new Foot.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.input.Sensors.IFoot);

                /** Foot down. */
                public down: boolean;

                /** Foot Hwf. */
                public Hwf?: (Imat4|null);

                /**
                 * Encodes the specified Foot message. Does not implicitly {@link message.input.Sensors.Foot.verify|verify} messages.
                 * @param message Foot message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.input.Sensors.IFoot, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Foot message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Foot
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.input.Sensors.Foot;
            }
        }
    }

    /** Namespace localisation. */
    namespace localisation {

        /** Properties of a Ball. */
        interface IBall {

            /** The ball's position measured in world space (rBWw) */
            position?: (Ivec2|null);

            /** Ball covariance */
            covariance?: (Imat2|null);
        }

        /** Represents a Ball. */
        class Ball implements IBall {

            /**
             * Constructs a new Ball.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.localisation.IBall);

            /** The ball's position measured in world space (rBWw) */
            public position?: (Ivec2|null);

            /** Ball covariance. */
            public covariance?: (Imat2|null);

            /**
             * Encodes the specified Ball message. Does not implicitly {@link message.localisation.Ball.verify|verify} messages.
             * @param message Ball message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.localisation.IBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Ball message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Ball
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.Ball;
        }

        /** Properties of a Field. */
        interface IField {

            /** Isometry2d position transform */
            position?: (Imat3|null);

            /** Covariance of (x, y, theta) */
            covariance?: (Imat3|null);
        }

        /** Represents a Field. */
        class Field implements IField {

            /**
             * Constructs a new Field.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.localisation.IField);

            /** Isometry2d position transform */
            public position?: (Imat3|null);

            /** Covariance of (x, y, theta) */
            public covariance?: (Imat3|null);

            /**
             * Encodes the specified Field message. Does not implicitly {@link message.localisation.Field.verify|verify} messages.
             * @param message Field message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.localisation.IField, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Field message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Field
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.Field;
        }

        /** Properties of a FilteredBall. */
        interface IFilteredBall {

            /** Ball's filtered cartesian position measured in torso space {t} from the torso {T} */
            rBTt?: (Ifvec3|null);

            /** Ball's filtered cartesian position measured in camera space {c} from the camera {C} */
            rBCc?: (Ifvec3|null);

            /** Ball's filtered cartesian position measured in torso space {t} from the camera {C} */
            rBCt?: (Ifvec3|null);

            /** Time the ball measurement was taken */
            timeOfMeasurement?: (google.protobuf.ITimestamp|null);
        }

        /** Represents a FilteredBall. */
        class FilteredBall implements IFilteredBall {

            /**
             * Constructs a new FilteredBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.localisation.IFilteredBall);

            /** Ball's filtered cartesian position measured in torso space {t} from the torso {T} */
            public rBTt?: (Ifvec3|null);

            /** Ball's filtered cartesian position measured in camera space {c} from the camera {C} */
            public rBCc?: (Ifvec3|null);

            /** Ball's filtered cartesian position measured in torso space {t} from the camera {C} */
            public rBCt?: (Ifvec3|null);

            /** Time the ball measurement was taken */
            public timeOfMeasurement?: (google.protobuf.ITimestamp|null);

            /**
             * Encodes the specified FilteredBall message. Does not implicitly {@link message.localisation.FilteredBall.verify|verify} messages.
             * @param message FilteredBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.localisation.IFilteredBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FilteredBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FilteredBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.FilteredBall;
        }

        /** Properties of a ResetBallHypotheses. */
        interface IResetBallHypotheses {

            /** ResetBallHypotheses hypotheses */
            hypotheses?: (message.localisation.ResetBallHypotheses.IBall[]|null);

            /** ResetBallHypotheses selfReset */
            selfReset?: (boolean|null);
        }

        /** Represents a ResetBallHypotheses. */
        class ResetBallHypotheses implements IResetBallHypotheses {

            /**
             * Constructs a new ResetBallHypotheses.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.localisation.IResetBallHypotheses);

            /** ResetBallHypotheses hypotheses. */
            public hypotheses: message.localisation.ResetBallHypotheses.IBall[];

            /** ResetBallHypotheses selfReset. */
            public selfReset: boolean;

            /**
             * Encodes the specified ResetBallHypotheses message. Does not implicitly {@link message.localisation.ResetBallHypotheses.verify|verify} messages.
             * @param message ResetBallHypotheses message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.localisation.IResetBallHypotheses, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ResetBallHypotheses message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ResetBallHypotheses
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.ResetBallHypotheses;
        }

        namespace ResetBallHypotheses {

            /** Properties of a Ball. */
            interface IBall {

                /** Estimate of the balls position in world space */
                rBWw?: (Ivec2|null);

                /** Ball covariance */
                covariance?: (Imat2|null);
            }

            /** Represents a Ball. */
            class Ball implements IBall {

                /**
                 * Constructs a new Ball.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.localisation.ResetBallHypotheses.IBall);

                /** Estimate of the balls position in world space */
                public rBWw?: (Ivec2|null);

                /** Ball covariance. */
                public covariance?: (Imat2|null);

                /**
                 * Encodes the specified Ball message. Does not implicitly {@link message.localisation.ResetBallHypotheses.Ball.verify|verify} messages.
                 * @param message Ball message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.localisation.ResetBallHypotheses.IBall, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Ball message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Ball
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.ResetBallHypotheses.Ball;
            }
        }

        /** Properties of a ResetRobotHypotheses. */
        interface IResetRobotHypotheses {

            /** ResetRobotHypotheses hypotheses */
            hypotheses?: (message.localisation.ResetRobotHypotheses.ISelf[]|null);
        }

        /** Represents a ResetRobotHypotheses. */
        class ResetRobotHypotheses implements IResetRobotHypotheses {

            /**
             * Constructs a new ResetRobotHypotheses.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.localisation.IResetRobotHypotheses);

            /** ResetRobotHypotheses hypotheses. */
            public hypotheses: message.localisation.ResetRobotHypotheses.ISelf[];

            /**
             * Encodes the specified ResetRobotHypotheses message. Does not implicitly {@link message.localisation.ResetRobotHypotheses.verify|verify} messages.
             * @param message ResetRobotHypotheses message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.localisation.IResetRobotHypotheses, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ResetRobotHypotheses message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ResetRobotHypotheses
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.ResetRobotHypotheses;
        }

        namespace ResetRobotHypotheses {

            /** Properties of a Self. */
            interface ISelf {

                /** Estimate of the robot's {x, y} position in field space */
                rTFf?: (Ivec2|null);

                /** Self covariance */
                covariance?: (Imat2|null);

                /** Self heading */
                heading?: (number|null);

                /** Self headingVar */
                headingVar?: (number|null);

                /** Self absoluteYaw */
                absoluteYaw?: (boolean|null);
            }

            /** Represents a Self. */
            class Self implements ISelf {

                /**
                 * Constructs a new Self.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.localisation.ResetRobotHypotheses.ISelf);

                /** Estimate of the robot's {x, y} position in field space */
                public rTFf?: (Ivec2|null);

                /** Self covariance. */
                public covariance?: (Imat2|null);

                /** Self heading. */
                public heading: number;

                /** Self headingVar. */
                public headingVar: number;

                /** Self absoluteYaw. */
                public absoluteYaw: boolean;

                /**
                 * Encodes the specified Self message. Does not implicitly {@link message.localisation.ResetRobotHypotheses.Self.verify|verify} messages.
                 * @param message Self message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.localisation.ResetRobotHypotheses.ISelf, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Self message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Self
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.localisation.ResetRobotHypotheses.Self;
            }
        }
    }

    /** Namespace motion. */
    namespace motion {

        /** Properties of an ExecuteGetup. */
        interface IExecuteGetup {
        }

        /** Represents an ExecuteGetup. */
        class ExecuteGetup implements IExecuteGetup {

            /**
             * Constructs a new ExecuteGetup.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IExecuteGetup);

            /**
             * Encodes the specified ExecuteGetup message. Does not implicitly {@link message.motion.ExecuteGetup.verify|verify} messages.
             * @param message ExecuteGetup message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IExecuteGetup, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an ExecuteGetup message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ExecuteGetup
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.ExecuteGetup;
        }

        /** Properties of a KillGetup. */
        interface IKillGetup {
        }

        /** Represents a KillGetup. */
        class KillGetup implements IKillGetup {

            /**
             * Constructs a new KillGetup.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IKillGetup);

            /**
             * Encodes the specified KillGetup message. Does not implicitly {@link message.motion.KillGetup.verify|verify} messages.
             * @param message KillGetup message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IKillGetup, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KillGetup message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KillGetup
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.KillGetup;
        }

        /** Properties of a HeadCommand. */
        interface IHeadCommand {

            /** Target head yaw in radians */
            yaw?: (number|null);

            /** Target head pitch in radians */
            pitch?: (number|null);

            /** If true, the yaw and pitch are interpreted in robot space, instead of IMU space */
            robotSpace?: (boolean|null);

            /** If true, smooth the goal angle using expoential smoothing */
            smooth?: (boolean|null);
        }

        /**
         * Tell the head where to look in world space.
         * This command is interpreted such that the robot will use IMU data to fixate
         * at these angles in the world even when rotating.
         *
         */
        class HeadCommand implements IHeadCommand {

            /**
             * Constructs a new HeadCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IHeadCommand);

            /** Target head yaw in radians */
            public yaw: number;

            /** Target head pitch in radians */
            public pitch: number;

            /** If true, the yaw and pitch are interpreted in robot space, instead of IMU space */
            public robotSpace: boolean;

            /** If true, smooth the goal angle using expoential smoothing */
            public smooth: boolean;

            /**
             * Encodes the specified HeadCommand message. Does not implicitly {@link message.motion.HeadCommand.verify|verify} messages.
             * @param message HeadCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IHeadCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a HeadCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns HeadCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.HeadCommand;
        }

        /** undefined */
        enum KickCommandType {
            NORMAL = 0,
            PENALTY = 1
        }

        /** Properties of a KickCommand. */
        interface IKickCommand {

            /** The point to kick */
            target?: (Ivec3|null);

            /** corresponding to the force of the kick */
            direction?: (Ivec3|null);

            /** KickCommand kickCommandType */
            kickCommandType?: (message.motion.KickCommandType|null);
        }

        /** Represents a KickCommand. */
        class KickCommand implements IKickCommand {

            /**
             * Constructs a new KickCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IKickCommand);

            /** The point to kick */
            public target?: (Ivec3|null);

            /** corresponding to the force of the kick */
            public direction?: (Ivec3|null);

            /** KickCommand kickCommandType. */
            public kickCommandType: message.motion.KickCommandType;

            /**
             * Encodes the specified KickCommand message. Does not implicitly {@link message.motion.KickCommand.verify|verify} messages.
             * @param message KickCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IKickCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.KickCommand;
        }

        /** Properties of a KickScriptCommand. */
        interface IKickScriptCommand {

            /** Leg to kick with */
            leg?: (number|null);

            /** KickScriptCommand type */
            type?: (message.motion.KickCommandType|null);
        }

        /** @author Trent Houliston */
        class KickScriptCommand implements IKickScriptCommand {

            /**
             * Constructs a new KickScriptCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IKickScriptCommand);

            /** Leg to kick with */
            public leg: number;

            /** KickScriptCommand type. */
            public type: message.motion.KickCommandType;

            /**
             * Encodes the specified KickScriptCommand message. Does not implicitly {@link message.motion.KickScriptCommand.verify|verify} messages.
             * @param message KickScriptCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IKickScriptCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickScriptCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickScriptCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.KickScriptCommand;
        }

        /** Properties of a KickFinished. */
        interface IKickFinished {
        }

        /** Represents a KickFinished. */
        class KickFinished implements IKickFinished {

            /**
             * Constructs a new KickFinished.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IKickFinished);

            /**
             * Encodes the specified KickFinished message. Does not implicitly {@link message.motion.KickFinished.verify|verify} messages.
             * @param message KickFinished message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IKickFinished, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickFinished message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickFinished
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.KickFinished;
        }

        /** Properties of a IKKickParams. */
        interface IIKKickParams {

            /** IKKickParams standHeight */
            standHeight?: (number|null);
        }

        /** Represents a IKKickParams. */
        class IKKickParams implements IIKKickParams {

            /**
             * Constructs a new IKKickParams.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IIKKickParams);

            /** IKKickParams standHeight. */
            public standHeight: number;

            /**
             * Encodes the specified IKKickParams message. Does not implicitly {@link message.motion.IKKickParams.verify|verify} messages.
             * @param message IKKickParams message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IIKKickParams, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a IKKickParams message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns IKKickParams
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.IKKickParams;
        }

        /** Properties of a WalkStarted. */
        interface IWalkStarted {
        }

        /** Represents a WalkStarted. */
        class WalkStarted implements IWalkStarted {

            /**
             * Constructs a new WalkStarted.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IWalkStarted);

            /**
             * Encodes the specified WalkStarted message. Does not implicitly {@link message.motion.WalkStarted.verify|verify} messages.
             * @param message WalkStarted message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IWalkStarted, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WalkStarted message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WalkStarted
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.WalkStarted;
        }

        /** Properties of a WalkStopped. */
        interface IWalkStopped {
        }

        /** Represents a WalkStopped. */
        class WalkStopped implements IWalkStopped {

            /**
             * Constructs a new WalkStopped.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IWalkStopped);

            /**
             * Encodes the specified WalkStopped message. Does not implicitly {@link message.motion.WalkStopped.verify|verify} messages.
             * @param message WalkStopped message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IWalkStopped, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WalkStopped message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WalkStopped
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.WalkStopped;
        }

        /** Properties of a WalkCommand. */
        interface IWalkCommand {

            /** Reservation identifier for servo control */
            subsumptionId?: (number|Long|null);

            /** required rotational velocity about the z axis. In torso space */
            command?: (Ivec3|null);
        }

        /** Represents a WalkCommand. */
        class WalkCommand implements IWalkCommand {

            /**
             * Constructs a new WalkCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IWalkCommand);

            /** Reservation identifier for servo control */
            public subsumptionId: (number|Long);

            /** required rotational velocity about the z axis. In torso space */
            public command?: (Ivec3|null);

            /**
             * Encodes the specified WalkCommand message. Does not implicitly {@link message.motion.WalkCommand.verify|verify} messages.
             * @param message WalkCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IWalkCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WalkCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WalkCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.WalkCommand;
        }

        /** Properties of a StopCommand. */
        interface IStopCommand {

            /** Reservation identifier for servo control */
            subsumptionId?: (number|Long|null);
        }

        /** Represents a StopCommand. */
        class StopCommand implements IStopCommand {

            /**
             * Constructs a new StopCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IStopCommand);

            /** Reservation identifier for servo control */
            public subsumptionId: (number|Long);

            /**
             * Encodes the specified StopCommand message. Does not implicitly {@link message.motion.StopCommand.verify|verify} messages.
             * @param message StopCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IStopCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a StopCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns StopCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.StopCommand;
        }

        /** Properties of a NewWalkCommand. */
        interface INewWalkCommand {

            /** NewWalkCommand velocityTarget */
            velocityTarget?: (Ivec3|null);
        }

        /** Represents a NewWalkCommand. */
        class NewWalkCommand implements INewWalkCommand {

            /**
             * Constructs a new NewWalkCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.INewWalkCommand);

            /** NewWalkCommand velocityTarget. */
            public velocityTarget?: (Ivec3|null);

            /**
             * Encodes the specified NewWalkCommand message. Does not implicitly {@link message.motion.NewWalkCommand.verify|verify} messages.
             * @param message NewWalkCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.INewWalkCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a NewWalkCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns NewWalkCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.NewWalkCommand;
        }

        /** Properties of an EnableWalkEngineCommand. */
        interface IEnableWalkEngineCommand {

            /** Reservation identifier for servo control */
            subsumptionId?: (number|Long|null);
        }

        /** Represents an EnableWalkEngineCommand. */
        class EnableWalkEngineCommand implements IEnableWalkEngineCommand {

            /**
             * Constructs a new EnableWalkEngineCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IEnableWalkEngineCommand);

            /** Reservation identifier for servo control */
            public subsumptionId: (number|Long);

            /**
             * Encodes the specified EnableWalkEngineCommand message. Does not implicitly {@link message.motion.EnableWalkEngineCommand.verify|verify} messages.
             * @param message EnableWalkEngineCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IEnableWalkEngineCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EnableWalkEngineCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EnableWalkEngineCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.EnableWalkEngineCommand;
        }

        /** Properties of a DisableWalkEngineCommand. */
        interface IDisableWalkEngineCommand {

            /** Reservation identifier for servo control */
            subsumptionId?: (number|Long|null);
        }

        /** Represents a DisableWalkEngineCommand. */
        class DisableWalkEngineCommand implements IDisableWalkEngineCommand {

            /**
             * Constructs a new DisableWalkEngineCommand.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.motion.IDisableWalkEngineCommand);

            /** Reservation identifier for servo control */
            public subsumptionId: (number|Long);

            /**
             * Encodes the specified DisableWalkEngineCommand message. Does not implicitly {@link message.motion.DisableWalkEngineCommand.verify|verify} messages.
             * @param message DisableWalkEngineCommand message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.motion.IDisableWalkEngineCommand, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DisableWalkEngineCommand message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DisableWalkEngineCommand
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.motion.DisableWalkEngineCommand;
        }
    }

    /** Namespace output. */
    namespace output {

        /** Properties of a CompressedImage. */
        interface ICompressedImage {

            /** see module::output::compressor::turbojpeg::Compressor::compress for full list we use */
            format?: (number|null);

            /** Number of pixels of {x, y} of the image */
            dimensions?: (Iuvec2|null);

            /** CompressedImage data */
            data?: (Uint8Array|null);

            /** CompressedImage id */
            id?: (number|null);

            /** Name of the camera that took the image */
            name?: (string|null);

            /** Time when the image was taken */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** World to camera Isometry3d transform */
            Hcw?: (Imat4|null);

            /** Lens information of the camera that took the image */
            lens?: (message.output.CompressedImage.ILens|null);
        }

        /** Represents a CompressedImage. */
        class CompressedImage implements ICompressedImage {

            /**
             * Constructs a new CompressedImage.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.output.ICompressedImage);

            /** see module::output::compressor::turbojpeg::Compressor::compress for full list we use */
            public format: number;

            /** Number of pixels of {x, y} of the image */
            public dimensions?: (Iuvec2|null);

            /** CompressedImage data. */
            public data: Uint8Array;

            /** CompressedImage id. */
            public id: number;

            /** Name of the camera that took the image */
            public name: string;

            /** Time when the image was taken */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** World to camera Isometry3d transform */
            public Hcw?: (Imat4|null);

            /** Lens information of the camera that took the image */
            public lens?: (message.output.CompressedImage.ILens|null);

            /**
             * Encodes the specified CompressedImage message. Does not implicitly {@link message.output.CompressedImage.verify|verify} messages.
             * @param message CompressedImage message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.output.ICompressedImage, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a CompressedImage message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns CompressedImage
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.output.CompressedImage;
        }

        namespace CompressedImage {

            /** Properties of a Lens. */
            interface ILens {

                /** Lens projection */
                projection?: (message.output.CompressedImage.Lens.Projection|null);

                /** Normalised focal length, in pixels / image width */
                focalLength?: (number|null);

                /** Angular diameter that the lens covers (the area that light hits on the sensor), in radians */
                fov?: (number|null);

                /** Normalised image centre offset - the number of pixels from centre to optical axis / image width */
                centre?: (Ifvec2|null);

                /** Lens k */
                k?: (Ifvec2|null);
            }

            /** Represents a Lens. */
            class Lens implements ILens {

                /**
                 * Constructs a new Lens.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.output.CompressedImage.ILens);

                /** Lens projection. */
                public projection: message.output.CompressedImage.Lens.Projection;

                /** Normalised focal length, in pixels / image width */
                public focalLength: number;

                /** Angular diameter that the lens covers (the area that light hits on the sensor), in radians */
                public fov: number;

                /** Normalised image centre offset - the number of pixels from centre to optical axis / image width */
                public centre?: (Ifvec2|null);

                /** Lens k. */
                public k?: (Ifvec2|null);

                /**
                 * Encodes the specified Lens message. Does not implicitly {@link message.output.CompressedImage.Lens.verify|verify} messages.
                 * @param message Lens message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.output.CompressedImage.ILens, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Lens message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Lens
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.output.CompressedImage.Lens;
            }

            namespace Lens {

                /** Projection enum. */
                enum Projection {
                    UNKNOWN = 0,
                    RECTILINEAR = 1,
                    EQUIDISTANT = 2,
                    EQUISOLID = 3
                }
            }
        }

        /** Properties of a Say. */
        interface ISay {

            /** Say message */
            message?: (string|null);
        }

        /** Represents a Say. */
        class Say implements ISay {

            /**
             * Constructs a new Say.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.output.ISay);

            /** Say message. */
            public message: string;

            /**
             * Encodes the specified Say message. Does not implicitly {@link message.output.Say.verify|verify} messages.
             * @param message Say message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.output.ISay, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Say message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Say
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.output.Say;
        }
    }

    /** Namespace planning. */
    namespace planning {

        /** Properties of a GetUpWhenFallen. */
        interface IGetUpWhenFallen {
        }

        /**
         * Emit a GetUpWhenFallen Task in order to tell the GetUpPlanner to start monitoring the robot's state and get it up if
         * it falls.
         */
        class GetUpWhenFallen implements IGetUpWhenFallen {

            /**
             * Constructs a new GetUpWhenFallen.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.IGetUpWhenFallen);

            /**
             * Encodes the specified GetUpWhenFallen message. Does not implicitly {@link message.planning.GetUpWhenFallen.verify|verify} messages.
             * @param message GetUpWhenFallen message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.IGetUpWhenFallen, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GetUpWhenFallen message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GetUpWhenFallen
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.GetUpWhenFallen;
        }

        /** Properties of a KickTo. */
        interface IKickTo {

            /** Torso to point to kick to in torso space */
            rPTt?: (Ifvec3|null);
        }

        /** Represents a KickTo. */
        class KickTo implements IKickTo {

            /**
             * Constructs a new KickTo.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.IKickTo);

            /** Torso to point to kick to in torso space */
            public rPTt?: (Ifvec3|null);

            /**
             * Encodes the specified KickTo message. Does not implicitly {@link message.planning.KickTo.verify|verify} messages.
             * @param message KickTo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.IKickTo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickTo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickTo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.KickTo;
        }

        /** Properties of a LookAround. */
        interface ILookAround {
        }

        /** Task requesting to look around the environment */
        class LookAround implements ILookAround {

            /**
             * Constructs a new LookAround.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.ILookAround);

            /**
             * Encodes the specified LookAround message. Does not implicitly {@link message.planning.LookAround.verify|verify} messages.
             * @param message LookAround message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.ILookAround, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LookAround message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LookAround
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.LookAround;
        }

        /** Properties of a RelaxWhenFalling. */
        interface IRelaxWhenFalling {
        }

        /**
         * Emit a RelaxWhenFalling Task in order to tell the planner to start monitoring the robot's state and if it decides
         * that the robot is falling over, relax all the servos to prevent damage.
         */
        class RelaxWhenFalling implements IRelaxWhenFalling {

            /**
             * Constructs a new RelaxWhenFalling.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.IRelaxWhenFalling);

            /**
             * Encodes the specified RelaxWhenFalling message. Does not implicitly {@link message.planning.RelaxWhenFalling.verify|verify} messages.
             * @param message RelaxWhenFalling message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.IRelaxWhenFalling, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RelaxWhenFalling message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RelaxWhenFalling
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.RelaxWhenFalling;
        }

        /** Properties of a WalkTo. */
        interface IWalkTo {

            /** The point to walk to, relative to the torso */
            rPTt?: (Ifvec3|null);
        }

        /** Plan a path to walk to the specified point */
        class WalkTo implements IWalkTo {

            /**
             * Constructs a new WalkTo.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.IWalkTo);

            /** The point to walk to, relative to the torso */
            public rPTt?: (Ifvec3|null);

            /**
             * Encodes the specified WalkTo message. Does not implicitly {@link message.planning.WalkTo.verify|verify} messages.
             * @param message WalkTo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.IWalkTo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WalkTo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WalkTo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.WalkTo;
        }

        /** Properties of a TurnOnSpot. */
        interface ITurnOnSpot {

            /** Direction to turn */
            clockwise?: (boolean|null);
        }

        /** Turn on the spot in the given direction */
        class TurnOnSpot implements ITurnOnSpot {

            /**
             * Constructs a new TurnOnSpot.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.ITurnOnSpot);

            /** Direction to turn */
            public clockwise: boolean;

            /**
             * Encodes the specified TurnOnSpot message. Does not implicitly {@link message.planning.TurnOnSpot.verify|verify} messages.
             * @param message TurnOnSpot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.ITurnOnSpot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TurnOnSpot message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TurnOnSpot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.TurnOnSpot;
        }

        /** Properties of a TurnAroundBall. */
        interface ITurnAroundBall {

            /** Direction to turn */
            clockwise?: (boolean|null);
        }

        /** Turn around the ball in the given direction */
        class TurnAroundBall implements ITurnAroundBall {

            /**
             * Constructs a new TurnAroundBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.planning.ITurnAroundBall);

            /** Direction to turn */
            public clockwise: boolean;

            /**
             * Encodes the specified TurnAroundBall message. Does not implicitly {@link message.planning.TurnAroundBall.verify|verify} messages.
             * @param message TurnAroundBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.planning.ITurnAroundBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TurnAroundBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TurnAroundBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.planning.TurnAroundBall;
        }
    }

    /** Namespace platform. */
    namespace platform {

        /** Properties of a RawSensors. */
        interface IRawSensors {

            /** Timestamp when our data were taken */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** The list of errors is here: https://emanual.robotis.com/docs/en/dxl/protocol1/#status-packetreturn-packet */
            platformErrorFlags?: (number|null);

            /** RawSensors ledPanel */
            ledPanel?: (message.platform.RawSensors.ILEDPanel|null);

            /** RawSensors headLed */
            headLed?: (message.platform.RawSensors.IHeadLED|null);

            /** RawSensors eyeLed */
            eyeLed?: (message.platform.RawSensors.IEyeLED|null);

            /** RawSensors buttons */
            buttons?: (message.platform.RawSensors.IButtons|null);

            /** Voltage for the whole robot, from the main battery, in Volts */
            voltage?: (number|null);

            /** here https://emanual.robotis.com/docs/en/platform/op2/getting_started/#acc_x-acc_y-acc_z */
            accelerometer?: (Ifvec3|null);

            /** here https://emanual.robotis.com/docs/en/platform/op2/getting_started/#gyro_x-gyro_y-gyro_z */
            gyroscope?: (Ifvec3|null);

            /** RawSensors fsr */
            fsr?: (message.platform.RawSensors.IFSRs|null);

            /** RawSensors servo */
            servo?: (message.platform.RawSensors.IServos|null);

            /** Ground truth from a simulator */
            odometryGroundTruth?: (message.platform.webots.IOdometryGroundTruth|null);
        }

        /** @author Trent Houliston */
        class RawSensors implements IRawSensors {

            /**
             * Constructs a new RawSensors.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IRawSensors);

            /** Timestamp when our data were taken */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** The list of errors is here: https://emanual.robotis.com/docs/en/dxl/protocol1/#status-packetreturn-packet */
            public platformErrorFlags: number;

            /** RawSensors ledPanel. */
            public ledPanel?: (message.platform.RawSensors.ILEDPanel|null);

            /** RawSensors headLed. */
            public headLed?: (message.platform.RawSensors.IHeadLED|null);

            /** RawSensors eyeLed. */
            public eyeLed?: (message.platform.RawSensors.IEyeLED|null);

            /** RawSensors buttons. */
            public buttons?: (message.platform.RawSensors.IButtons|null);

            /** Voltage for the whole robot, from the main battery, in Volts */
            public voltage: number;

            /** here https://emanual.robotis.com/docs/en/platform/op2/getting_started/#acc_x-acc_y-acc_z */
            public accelerometer?: (Ifvec3|null);

            /** here https://emanual.robotis.com/docs/en/platform/op2/getting_started/#gyro_x-gyro_y-gyro_z */
            public gyroscope?: (Ifvec3|null);

            /** RawSensors fsr. */
            public fsr?: (message.platform.RawSensors.IFSRs|null);

            /** RawSensors servo. */
            public servo?: (message.platform.RawSensors.IServos|null);

            /** Ground truth from a simulator */
            public odometryGroundTruth?: (message.platform.webots.IOdometryGroundTruth|null);

            /**
             * Encodes the specified RawSensors message. Does not implicitly {@link message.platform.RawSensors.verify|verify} messages.
             * @param message RawSensors message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IRawSensors, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a RawSensors message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns RawSensors
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors;
        }

        namespace RawSensors {

            /** See https://emanual.robotis.com/docs/en/dxl/protocol1/#protocol */
            enum Error {
                OK = 0,
                INPUT_VOLTAGE = 1,
                ANGLE_LIMIT = 2,
                OVERHEATING = 4,
                RANGE = 8,
                CHECKSUM = 16,
                OVERLOAD = 32,
                INSTRUCTION = 64,
                CORRUPT_DATA = 128,
                TIMEOUT = 256
            }

            /** Properties of a LEDPanel. */
            interface ILEDPanel {

                /** LEDPanel led2 */
                led2?: (boolean|null);

                /** LEDPanel led3 */
                led3?: (boolean|null);

                /** LEDPanel led4 */
                led4?: (boolean|null);
            }

            /** Represents a LEDPanel. */
            class LEDPanel implements ILEDPanel {

                /**
                 * Constructs a new LEDPanel.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.ILEDPanel);

                /** LEDPanel led2. */
                public led2: boolean;

                /** LEDPanel led3. */
                public led3: boolean;

                /** LEDPanel led4. */
                public led4: boolean;

                /**
                 * Encodes the specified LEDPanel message. Does not implicitly {@link message.platform.RawSensors.LEDPanel.verify|verify} messages.
                 * @param message LEDPanel message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.ILEDPanel, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a LEDPanel message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns LEDPanel
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.LEDPanel;
            }

            /** Properties of a HeadLED. */
            interface IHeadLED {

                /** Encode as 0x00 0xRR 0xGG 0xBB */
                RGB?: (number|null);
            }

            /** Represents a HeadLED. */
            class HeadLED implements IHeadLED {

                /**
                 * Constructs a new HeadLED.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IHeadLED);

                /** Encode as 0x00 0xRR 0xGG 0xBB */
                public RGB: number;

                /**
                 * Encodes the specified HeadLED message. Does not implicitly {@link message.platform.RawSensors.HeadLED.verify|verify} messages.
                 * @param message HeadLED message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IHeadLED, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a HeadLED message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns HeadLED
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.HeadLED;
            }

            /** Properties of an EyeLED. */
            interface IEyeLED {

                /** Encode as 0x00 0xRR 0xGG 0xBB */
                RGB?: (number|null);
            }

            /** Represents an EyeLED. */
            class EyeLED implements IEyeLED {

                /**
                 * Constructs a new EyeLED.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IEyeLED);

                /** Encode as 0x00 0xRR 0xGG 0xBB */
                public RGB: number;

                /**
                 * Encodes the specified EyeLED message. Does not implicitly {@link message.platform.RawSensors.EyeLED.verify|verify} messages.
                 * @param message EyeLED message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IEyeLED, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an EyeLED message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns EyeLED
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.EyeLED;
            }

            /** Properties of a Buttons. */
            interface IButtons {

                /** Buttons left */
                left?: (boolean|null);

                /** Buttons middle */
                middle?: (boolean|null);
            }

            /** Represents a Buttons. */
            class Buttons implements IButtons {

                /**
                 * Constructs a new Buttons.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IButtons);

                /** Buttons left. */
                public left: boolean;

                /** Buttons middle. */
                public middle: boolean;

                /**
                 * Encodes the specified Buttons message. Does not implicitly {@link message.platform.RawSensors.Buttons.verify|verify} messages.
                 * @param message Buttons message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IButtons, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Buttons message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Buttons
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.Buttons;
            }

            /** Properties of a FSR. */
            interface IFSR {

                /** FSR fsr1 */
                fsr1?: (number|null);

                /** FSR fsr2 */
                fsr2?: (number|null);

                /** FSR fsr3 */
                fsr3?: (number|null);

                /** FSR fsr4 */
                fsr4?: (number|null);

                /** FSR centreX */
                centreX?: (number|null);

                /** FSR centreY */
                centreY?: (number|null);

                /** FSR errorFlags */
                errorFlags?: (number|null);
            }

            /** There are four FSRs per foot, so this message is for one foot */
            class FSR implements IFSR {

                /**
                 * Constructs a new FSR.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IFSR);

                /** FSR fsr1. */
                public fsr1: number;

                /** FSR fsr2. */
                public fsr2: number;

                /** FSR fsr3. */
                public fsr3: number;

                /** FSR fsr4. */
                public fsr4: number;

                /** FSR centreX. */
                public centreX: number;

                /** FSR centreY. */
                public centreY: number;

                /** FSR errorFlags. */
                public errorFlags: number;

                /**
                 * Encodes the specified FSR message. Does not implicitly {@link message.platform.RawSensors.FSR.verify|verify} messages.
                 * @param message FSR message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IFSR, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a FSR message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns FSR
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.FSR;
            }

            /** Properties of a FSRs. */
            interface IFSRs {

                /** Left foot */
                left?: (message.platform.RawSensors.IFSR|null);

                /** Right foot */
                right?: (message.platform.RawSensors.IFSR|null);
            }

            /** Represents a FSRs. */
            class FSRs implements IFSRs {

                /**
                 * Constructs a new FSRs.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IFSRs);

                /** Left foot */
                public left?: (message.platform.RawSensors.IFSR|null);

                /** Right foot */
                public right?: (message.platform.RawSensors.IFSR|null);

                /**
                 * Encodes the specified FSRs message. Does not implicitly {@link message.platform.RawSensors.FSRs.verify|verify} messages.
                 * @param message FSRs message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IFSRs, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a FSRs message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns FSRs
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.FSRs;
            }

            /** Properties of a Servo. */
            interface IServo {

                /** https://emanual.robotis.com/docs/en/dxl/mx/mx-106/#alarm-led17-shutdown18 */
                errorFlags?: (number|null);

                /** Servo torqueEnabled */
                torqueEnabled?: (boolean|null);

                /** Proportional gain. In proportion to the servo's position error. Gain values are in range 0~100 */
                pGain?: (number|null);

                /** range 0~100 */
                iGain?: (number|null);

                /** range 0~100 */
                dGain?: (number|null);

                /** Alarm is triggered as set in Alarm LED/Shutdown */
                goalPosition?: (number|null);

                /** What dynamixel servos call velocity. In radians per second */
                movingSpeed?: (number|null);

                /** Used to set the servo on or off. Typically either 0 (off) or 100 (on). Not really in any unit */
                torque?: (number|null);

                /** In radians */
                presentPosition?: (number|null);

                /** In radians per second */
                presentSpeed?: (number|null);

                /** is clockwise */
                load?: (number|null);

                /** returned as ‘1’ and Alarm is triggered and set the address 17 and set 1 to the Bit 0 of the address 18 */
                voltage?: (number|null);

                /** In degrees Celsius */
                temperature?: (number|null);
            }

            /** Ref: https://emanual.robotis.com/docs/en/dxl/mx/mx-64/ ; https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ */
            class Servo implements IServo {

                /**
                 * Constructs a new Servo.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IServo);

                /** https://emanual.robotis.com/docs/en/dxl/mx/mx-106/#alarm-led17-shutdown18 */
                public errorFlags: number;

                /** Servo torqueEnabled. */
                public torqueEnabled: boolean;

                /** Proportional gain. In proportion to the servo's position error. Gain values are in range 0~100 */
                public pGain: number;

                /** range 0~100 */
                public iGain: number;

                /** range 0~100 */
                public dGain: number;

                /** Alarm is triggered as set in Alarm LED/Shutdown */
                public goalPosition: number;

                /** What dynamixel servos call velocity. In radians per second */
                public movingSpeed: number;

                /** Used to set the servo on or off. Typically either 0 (off) or 100 (on). Not really in any unit */
                public torque: number;

                /** In radians */
                public presentPosition: number;

                /** In radians per second */
                public presentSpeed: number;

                /** is clockwise */
                public load: number;

                /** returned as ‘1’ and Alarm is triggered and set the address 17 and set 1 to the Bit 0 of the address 18 */
                public voltage: number;

                /** In degrees Celsius */
                public temperature: number;

                /**
                 * Encodes the specified Servo message. Does not implicitly {@link message.platform.RawSensors.Servo.verify|verify} messages.
                 * @param message Servo message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IServo, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Servo message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Servo
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.Servo;
            }

            /** Properties of a Servos. */
            interface IServos {

                /** Servos rShoulderPitch */
                rShoulderPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lShoulderPitch */
                lShoulderPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rShoulderRoll */
                rShoulderRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lShoulderRoll */
                lShoulderRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos rElbow */
                rElbow?: (message.platform.RawSensors.IServo|null);

                /** Servos lElbow */
                lElbow?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipYaw */
                rHipYaw?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipYaw */
                lHipYaw?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipRoll */
                rHipRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipRoll */
                lHipRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipPitch */
                rHipPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipPitch */
                lHipPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rKnee */
                rKnee?: (message.platform.RawSensors.IServo|null);

                /** Servos lKnee */
                lKnee?: (message.platform.RawSensors.IServo|null);

                /** Servos rAnklePitch */
                rAnklePitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lAnklePitch */
                lAnklePitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rAnkleRoll */
                rAnkleRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lAnkleRoll */
                lAnkleRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos headPan */
                headPan?: (message.platform.RawSensors.IServo|null);

                /** Servos headTilt */
                headTilt?: (message.platform.RawSensors.IServo|null);
            }

            /** Set of all the servos on the robot */
            class Servos implements IServos {

                /**
                 * Constructs a new Servos.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.RawSensors.IServos);

                /** Servos rShoulderPitch. */
                public rShoulderPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lShoulderPitch. */
                public lShoulderPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rShoulderRoll. */
                public rShoulderRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lShoulderRoll. */
                public lShoulderRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos rElbow. */
                public rElbow?: (message.platform.RawSensors.IServo|null);

                /** Servos lElbow. */
                public lElbow?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipYaw. */
                public rHipYaw?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipYaw. */
                public lHipYaw?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipRoll. */
                public rHipRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipRoll. */
                public lHipRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos rHipPitch. */
                public rHipPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lHipPitch. */
                public lHipPitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rKnee. */
                public rKnee?: (message.platform.RawSensors.IServo|null);

                /** Servos lKnee. */
                public lKnee?: (message.platform.RawSensors.IServo|null);

                /** Servos rAnklePitch. */
                public rAnklePitch?: (message.platform.RawSensors.IServo|null);

                /** Servos lAnklePitch. */
                public lAnklePitch?: (message.platform.RawSensors.IServo|null);

                /** Servos rAnkleRoll. */
                public rAnkleRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos lAnkleRoll. */
                public lAnkleRoll?: (message.platform.RawSensors.IServo|null);

                /** Servos headPan. */
                public headPan?: (message.platform.RawSensors.IServo|null);

                /** Servos headTilt. */
                public headTilt?: (message.platform.RawSensors.IServo|null);

                /**
                 * Encodes the specified Servos message. Does not implicitly {@link message.platform.RawSensors.Servos.verify|verify} messages.
                 * @param message Servos message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.RawSensors.IServos, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Servos message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Servos
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.RawSensors.Servos;
            }
        }

        /** Properties of a ButtonLeftDown. */
        interface IButtonLeftDown {
        }

        /** Represents a ButtonLeftDown. */
        class ButtonLeftDown implements IButtonLeftDown {

            /**
             * Constructs a new ButtonLeftDown.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IButtonLeftDown);

            /**
             * Encodes the specified ButtonLeftDown message. Does not implicitly {@link message.platform.ButtonLeftDown.verify|verify} messages.
             * @param message ButtonLeftDown message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IButtonLeftDown, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ButtonLeftDown message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ButtonLeftDown
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.ButtonLeftDown;
        }

        /** Properties of a ButtonLeftUp. */
        interface IButtonLeftUp {
        }

        /** Represents a ButtonLeftUp. */
        class ButtonLeftUp implements IButtonLeftUp {

            /**
             * Constructs a new ButtonLeftUp.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IButtonLeftUp);

            /**
             * Encodes the specified ButtonLeftUp message. Does not implicitly {@link message.platform.ButtonLeftUp.verify|verify} messages.
             * @param message ButtonLeftUp message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IButtonLeftUp, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ButtonLeftUp message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ButtonLeftUp
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.ButtonLeftUp;
        }

        /** Properties of a ButtonMiddleDown. */
        interface IButtonMiddleDown {
        }

        /** Represents a ButtonMiddleDown. */
        class ButtonMiddleDown implements IButtonMiddleDown {

            /**
             * Constructs a new ButtonMiddleDown.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IButtonMiddleDown);

            /**
             * Encodes the specified ButtonMiddleDown message. Does not implicitly {@link message.platform.ButtonMiddleDown.verify|verify} messages.
             * @param message ButtonMiddleDown message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IButtonMiddleDown, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ButtonMiddleDown message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ButtonMiddleDown
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.ButtonMiddleDown;
        }

        /** Properties of a ButtonMiddleUp. */
        interface IButtonMiddleUp {
        }

        /** Represents a ButtonMiddleUp. */
        class ButtonMiddleUp implements IButtonMiddleUp {

            /**
             * Constructs a new ButtonMiddleUp.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IButtonMiddleUp);

            /**
             * Encodes the specified ButtonMiddleUp message. Does not implicitly {@link message.platform.ButtonMiddleUp.verify|verify} messages.
             * @param message ButtonMiddleUp message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IButtonMiddleUp, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ButtonMiddleUp message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ButtonMiddleUp
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.ButtonMiddleUp;
        }

        /** Properties of a ResetWebotsServos. */
        interface IResetWebotsServos {
        }

        /** Represents a ResetWebotsServos. */
        class ResetWebotsServos implements IResetWebotsServos {

            /**
             * Constructs a new ResetWebotsServos.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.platform.IResetWebotsServos);

            /**
             * Encodes the specified ResetWebotsServos message. Does not implicitly {@link message.platform.ResetWebotsServos.verify|verify} messages.
             * @param message ResetWebotsServos message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.platform.IResetWebotsServos, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ResetWebotsServos message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ResetWebotsServos
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.ResetWebotsServos;
        }

        /** Namespace webots. */
        namespace webots {

            /** Properties of a Vector3. */
            interface IVector3 {

                /** Vector3 X */
                X?: (number|null);

                /** Vector3 Y */
                Y?: (number|null);

                /** Vector3 Z */
                Z?: (number|null);
            }

            /** Represents a Vector3. */
            class Vector3 implements IVector3 {

                /**
                 * Constructs a new Vector3.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IVector3);

                /** Vector3 X. */
                public X: number;

                /** Vector3 Y. */
                public Y: number;

                /** Vector3 Z. */
                public Z: number;

                /**
                 * Encodes the specified Vector3 message. Does not implicitly {@link message.platform.webots.Vector3.verify|verify} messages.
                 * @param message Vector3 message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IVector3, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Vector3 message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Vector3
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.Vector3;
            }

            /** Properties of a PositionSensorMeasurement. */
            interface IPositionSensorMeasurement {

                /** PositionSensorMeasurement name */
                name?: (string|null);

                /** PositionSensorMeasurement value */
                value?: (number|null);
            }

            /** Represents a PositionSensorMeasurement. */
            class PositionSensorMeasurement implements IPositionSensorMeasurement {

                /**
                 * Constructs a new PositionSensorMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IPositionSensorMeasurement);

                /** PositionSensorMeasurement name. */
                public name: string;

                /** PositionSensorMeasurement value. */
                public value: number;

                /**
                 * Encodes the specified PositionSensorMeasurement message. Does not implicitly {@link message.platform.webots.PositionSensorMeasurement.verify|verify} messages.
                 * @param message PositionSensorMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IPositionSensorMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a PositionSensorMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns PositionSensorMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.PositionSensorMeasurement;
            }

            /** Properties of an AccelerometerMeasurement. */
            interface IAccelerometerMeasurement {

                /** AccelerometerMeasurement name */
                name?: (string|null);

                /** AccelerometerMeasurement value */
                value?: (message.platform.webots.IVector3|null);
            }

            /** Represents an AccelerometerMeasurement. */
            class AccelerometerMeasurement implements IAccelerometerMeasurement {

                /**
                 * Constructs a new AccelerometerMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IAccelerometerMeasurement);

                /** AccelerometerMeasurement name. */
                public name: string;

                /** AccelerometerMeasurement value. */
                public value?: (message.platform.webots.IVector3|null);

                /**
                 * Encodes the specified AccelerometerMeasurement message. Does not implicitly {@link message.platform.webots.AccelerometerMeasurement.verify|verify} messages.
                 * @param message AccelerometerMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IAccelerometerMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an AccelerometerMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns AccelerometerMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.AccelerometerMeasurement;
            }

            /** Properties of a GyroMeasurement. */
            interface IGyroMeasurement {

                /** GyroMeasurement name */
                name?: (string|null);

                /** GyroMeasurement value */
                value?: (message.platform.webots.IVector3|null);
            }

            /** Represents a GyroMeasurement. */
            class GyroMeasurement implements IGyroMeasurement {

                /**
                 * Constructs a new GyroMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IGyroMeasurement);

                /** GyroMeasurement name. */
                public name: string;

                /** GyroMeasurement value. */
                public value?: (message.platform.webots.IVector3|null);

                /**
                 * Encodes the specified GyroMeasurement message. Does not implicitly {@link message.platform.webots.GyroMeasurement.verify|verify} messages.
                 * @param message GyroMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IGyroMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a GyroMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns GyroMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.GyroMeasurement;
            }

            /** Properties of a BumperMeasurement. */
            interface IBumperMeasurement {

                /** BumperMeasurement name */
                name?: (string|null);

                /** BumperMeasurement value */
                value?: (boolean|null);
            }

            /** Represents a BumperMeasurement. */
            class BumperMeasurement implements IBumperMeasurement {

                /**
                 * Constructs a new BumperMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IBumperMeasurement);

                /** BumperMeasurement name. */
                public name: string;

                /** BumperMeasurement value. */
                public value: boolean;

                /**
                 * Encodes the specified BumperMeasurement message. Does not implicitly {@link message.platform.webots.BumperMeasurement.verify|verify} messages.
                 * @param message BumperMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IBumperMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a BumperMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns BumperMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.BumperMeasurement;
            }

            /** Properties of a ForceMeasurement. */
            interface IForceMeasurement {

                /** ForceMeasurement name */
                name?: (string|null);

                /** ForceMeasurement value */
                value?: (number|null);
            }

            /** Represents a ForceMeasurement. */
            class ForceMeasurement implements IForceMeasurement {

                /**
                 * Constructs a new ForceMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IForceMeasurement);

                /** ForceMeasurement name. */
                public name: string;

                /** ForceMeasurement value. */
                public value: number;

                /**
                 * Encodes the specified ForceMeasurement message. Does not implicitly {@link message.platform.webots.ForceMeasurement.verify|verify} messages.
                 * @param message ForceMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IForceMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a ForceMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ForceMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.ForceMeasurement;
            }

            /** Properties of a Force3DMeasurement. */
            interface IForce3DMeasurement {

                /** Force3DMeasurement name */
                name?: (string|null);

                /** Force3DMeasurement value */
                value?: (message.platform.webots.IVector3|null);
            }

            /** Represents a Force3DMeasurement. */
            class Force3DMeasurement implements IForce3DMeasurement {

                /**
                 * Constructs a new Force3DMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IForce3DMeasurement);

                /** Force3DMeasurement name. */
                public name: string;

                /** Force3DMeasurement value. */
                public value?: (message.platform.webots.IVector3|null);

                /**
                 * Encodes the specified Force3DMeasurement message. Does not implicitly {@link message.platform.webots.Force3DMeasurement.verify|verify} messages.
                 * @param message Force3DMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IForce3DMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Force3DMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Force3DMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.Force3DMeasurement;
            }

            /** Properties of a Force6DMeasurement. */
            interface IForce6DMeasurement {

                /** Force6DMeasurement name */
                name?: (string|null);

                /** Force6DMeasurement force */
                force?: (message.platform.webots.IVector3|null);

                /** Force6DMeasurement torque */
                torque?: (message.platform.webots.IVector3|null);
            }

            /** Represents a Force6DMeasurement. */
            class Force6DMeasurement implements IForce6DMeasurement {

                /**
                 * Constructs a new Force6DMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IForce6DMeasurement);

                /** Force6DMeasurement name. */
                public name: string;

                /** Force6DMeasurement force. */
                public force?: (message.platform.webots.IVector3|null);

                /** Force6DMeasurement torque. */
                public torque?: (message.platform.webots.IVector3|null);

                /**
                 * Encodes the specified Force6DMeasurement message. Does not implicitly {@link message.platform.webots.Force6DMeasurement.verify|verify} messages.
                 * @param message Force6DMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IForce6DMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Force6DMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Force6DMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.Force6DMeasurement;
            }

            /** Properties of a CameraMeasurement. */
            interface ICameraMeasurement {

                /** CameraMeasurement name */
                name?: (string|null);

                /** CameraMeasurement width */
                width?: (number|null);

                /** CameraMeasurement height */
                height?: (number|null);

                /** CameraMeasurement quality */
                quality?: (number|null);

                /** CameraMeasurement image */
                image?: (Uint8Array|null);
            }

            /** Represents a CameraMeasurement. */
            class CameraMeasurement implements ICameraMeasurement {

                /**
                 * Constructs a new CameraMeasurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.ICameraMeasurement);

                /** CameraMeasurement name. */
                public name: string;

                /** CameraMeasurement width. */
                public width: number;

                /** CameraMeasurement height. */
                public height: number;

                /** CameraMeasurement quality. */
                public quality: number;

                /** CameraMeasurement image. */
                public image: Uint8Array;

                /**
                 * Encodes the specified CameraMeasurement message. Does not implicitly {@link message.platform.webots.CameraMeasurement.verify|verify} messages.
                 * @param message CameraMeasurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.ICameraMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a CameraMeasurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns CameraMeasurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.CameraMeasurement;
            }

            /** Properties of a Message. */
            interface IMessage {

                /** Message messageType */
                messageType?: (message.platform.webots.Message.MessageType|null);

                /** Message text */
                text?: (string|null);
            }

            /** Represents a Message. */
            class Message implements IMessage {

                /**
                 * Constructs a new Message.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMessage);

                /** Message messageType. */
                public messageType: message.platform.webots.Message.MessageType;

                /** Message text. */
                public text: string;

                /**
                 * Encodes the specified Message message. Does not implicitly {@link message.platform.webots.Message.verify|verify} messages.
                 * @param message Message message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMessage, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Message message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Message
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.Message;
            }

            namespace Message {

                /** MessageType enum. */
                enum MessageType {
                    ERROR_MESSAGE = 0,
                    WARNING_MESSAGE = 1
                }
            }

            /** Properties of an OdometryGroundTruth. */
            interface IOdometryGroundTruth {

                /** Indicates if this message exists */
                exists?: (boolean|null);

                /** It measures the world in robot space */
                Htw?: (Imat4|null);
            }

            /** NUbots specific data sent from our own controllers rather than the official RoboCup Webots controller */
            class OdometryGroundTruth implements IOdometryGroundTruth {

                /**
                 * Constructs a new OdometryGroundTruth.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IOdometryGroundTruth);

                /** Indicates if this message exists */
                public exists: boolean;

                /** It measures the world in robot space */
                public Htw?: (Imat4|null);

                /**
                 * Encodes the specified OdometryGroundTruth message. Does not implicitly {@link message.platform.webots.OdometryGroundTruth.verify|verify} messages.
                 * @param message OdometryGroundTruth message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IOdometryGroundTruth, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an OdometryGroundTruth message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns OdometryGroundTruth
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.OdometryGroundTruth;
            }

            /** Properties of a VisionGroundTruth. */
            interface IVisionGroundTruth {

                /** Indicates if this message exists */
                exists?: (boolean|null);

                /** Vector from the world to the ball in world space. */
                rBWw?: (Ifvec3|null);
            }

            /** Represents a VisionGroundTruth. */
            class VisionGroundTruth implements IVisionGroundTruth {

                /**
                 * Constructs a new VisionGroundTruth.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IVisionGroundTruth);

                /** Indicates if this message exists */
                public exists: boolean;

                /** Vector from the world to the ball in world space. */
                public rBWw?: (Ifvec3|null);

                /**
                 * Encodes the specified VisionGroundTruth message. Does not implicitly {@link message.platform.webots.VisionGroundTruth.verify|verify} messages.
                 * @param message VisionGroundTruth message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IVisionGroundTruth, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a VisionGroundTruth message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns VisionGroundTruth
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.VisionGroundTruth;
            }

            /** Properties of a SensorMeasurements. */
            interface ISensorMeasurements {

                /** SensorMeasurements time */
                time?: (number|null);

                /** SensorMeasurements realTime */
                realTime?: (number|Long|null);

                /** SensorMeasurements messages */
                messages?: (message.platform.webots.IMessage[]|null);

                /** SensorMeasurements accelerometers */
                accelerometers?: (message.platform.webots.IAccelerometerMeasurement[]|null);

                /** SensorMeasurements bumpers */
                bumpers?: (message.platform.webots.IBumperMeasurement[]|null);

                /** SensorMeasurements cameras */
                cameras?: (message.platform.webots.ICameraMeasurement[]|null);

                /** SensorMeasurements forces */
                forces?: (message.platform.webots.IForceMeasurement[]|null);

                /** SensorMeasurements force3ds */
                force3ds?: (message.platform.webots.IForce3DMeasurement[]|null);

                /** SensorMeasurements force6ds */
                force6ds?: (message.platform.webots.IForce6DMeasurement[]|null);

                /** SensorMeasurements gyros */
                gyros?: (message.platform.webots.IGyroMeasurement[]|null);

                /** SensorMeasurements positionSensors */
                positionSensors?: (message.platform.webots.IPositionSensorMeasurement[]|null);

                /** SensorMeasurements odometryGroundTruth */
                odometryGroundTruth?: (message.platform.webots.IOdometryGroundTruth|null);

                /** SensorMeasurements visionGroundTruth */
                visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);
            }

            /** Represents a SensorMeasurements. */
            class SensorMeasurements implements ISensorMeasurements {

                /**
                 * Constructs a new SensorMeasurements.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.ISensorMeasurements);

                /** SensorMeasurements time. */
                public time: number;

                /** SensorMeasurements realTime. */
                public realTime: (number|Long);

                /** SensorMeasurements messages. */
                public messages: message.platform.webots.IMessage[];

                /** SensorMeasurements accelerometers. */
                public accelerometers: message.platform.webots.IAccelerometerMeasurement[];

                /** SensorMeasurements bumpers. */
                public bumpers: message.platform.webots.IBumperMeasurement[];

                /** SensorMeasurements cameras. */
                public cameras: message.platform.webots.ICameraMeasurement[];

                /** SensorMeasurements forces. */
                public forces: message.platform.webots.IForceMeasurement[];

                /** SensorMeasurements force3ds. */
                public force3ds: message.platform.webots.IForce3DMeasurement[];

                /** SensorMeasurements force6ds. */
                public force6ds: message.platform.webots.IForce6DMeasurement[];

                /** SensorMeasurements gyros. */
                public gyros: message.platform.webots.IGyroMeasurement[];

                /** SensorMeasurements positionSensors. */
                public positionSensors: message.platform.webots.IPositionSensorMeasurement[];

                /** SensorMeasurements odometryGroundTruth. */
                public odometryGroundTruth?: (message.platform.webots.IOdometryGroundTruth|null);

                /** SensorMeasurements visionGroundTruth. */
                public visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);

                /**
                 * Encodes the specified SensorMeasurements message. Does not implicitly {@link message.platform.webots.SensorMeasurements.verify|verify} messages.
                 * @param message SensorMeasurements message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.ISensorMeasurements, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SensorMeasurements message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SensorMeasurements
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.SensorMeasurements;
            }

            /** Properties of a MotorPosition. */
            interface IMotorPosition {

                /** MotorPosition name */
                name?: (string|null);

                /** MotorPosition position */
                position?: (number|null);
            }

            /** Represents a MotorPosition. */
            class MotorPosition implements IMotorPosition {

                /**
                 * Constructs a new MotorPosition.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMotorPosition);

                /** MotorPosition name. */
                public name: string;

                /** MotorPosition position. */
                public position: number;

                /**
                 * Encodes the specified MotorPosition message. Does not implicitly {@link message.platform.webots.MotorPosition.verify|verify} messages.
                 * @param message MotorPosition message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMotorPosition, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MotorPosition message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MotorPosition
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.MotorPosition;
            }

            /** Properties of a MotorVelocity. */
            interface IMotorVelocity {

                /** MotorVelocity name */
                name?: (string|null);

                /** MotorVelocity velocity */
                velocity?: (number|null);
            }

            /** Represents a MotorVelocity. */
            class MotorVelocity implements IMotorVelocity {

                /**
                 * Constructs a new MotorVelocity.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMotorVelocity);

                /** MotorVelocity name. */
                public name: string;

                /** MotorVelocity velocity. */
                public velocity: number;

                /**
                 * Encodes the specified MotorVelocity message. Does not implicitly {@link message.platform.webots.MotorVelocity.verify|verify} messages.
                 * @param message MotorVelocity message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMotorVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MotorVelocity message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MotorVelocity
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.MotorVelocity;
            }

            /** Properties of a MotorForce. */
            interface IMotorForce {

                /** MotorForce name */
                name?: (string|null);

                /** MotorForce force */
                force?: (number|null);
            }

            /** Represents a MotorForce. */
            class MotorForce implements IMotorForce {

                /**
                 * Constructs a new MotorForce.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMotorForce);

                /** MotorForce name. */
                public name: string;

                /** MotorForce force. */
                public force: number;

                /**
                 * Encodes the specified MotorForce message. Does not implicitly {@link message.platform.webots.MotorForce.verify|verify} messages.
                 * @param message MotorForce message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMotorForce, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MotorForce message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MotorForce
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.MotorForce;
            }

            /** Properties of a MotorTorque. */
            interface IMotorTorque {

                /** MotorTorque name */
                name?: (string|null);

                /** MotorTorque torque */
                torque?: (number|null);
            }

            /** Represents a MotorTorque. */
            class MotorTorque implements IMotorTorque {

                /**
                 * Constructs a new MotorTorque.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMotorTorque);

                /** MotorTorque name. */
                public name: string;

                /** MotorTorque torque. */
                public torque: number;

                /**
                 * Encodes the specified MotorTorque message. Does not implicitly {@link message.platform.webots.MotorTorque.verify|verify} messages.
                 * @param message MotorTorque message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMotorTorque, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MotorTorque message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MotorTorque
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.MotorTorque;
            }

            /** Properties of a MotorPID. */
            interface IMotorPID {

                /** MotorPID name */
                name?: (string|null);

                /** MotorPID PID */
                PID?: (message.platform.webots.IVector3|null);
            }

            /** Represents a MotorPID. */
            class MotorPID implements IMotorPID {

                /**
                 * Constructs a new MotorPID.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IMotorPID);

                /** MotorPID name. */
                public name: string;

                /** MotorPID PID. */
                public PID?: (message.platform.webots.IVector3|null);

                /**
                 * Encodes the specified MotorPID message. Does not implicitly {@link message.platform.webots.MotorPID.verify|verify} messages.
                 * @param message MotorPID message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IMotorPID, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a MotorPID message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns MotorPID
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.MotorPID;
            }

            /** Properties of a SensorTimeStep. */
            interface ISensorTimeStep {

                /** SensorTimeStep name */
                name?: (string|null);

                /** SensorTimeStep timeStep */
                timeStep?: (number|null);
            }

            /** Represents a SensorTimeStep. */
            class SensorTimeStep implements ISensorTimeStep {

                /**
                 * Constructs a new SensorTimeStep.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.ISensorTimeStep);

                /** SensorTimeStep name. */
                public name: string;

                /** SensorTimeStep timeStep. */
                public timeStep: number;

                /**
                 * Encodes the specified SensorTimeStep message. Does not implicitly {@link message.platform.webots.SensorTimeStep.verify|verify} messages.
                 * @param message SensorTimeStep message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.ISensorTimeStep, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SensorTimeStep message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SensorTimeStep
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.SensorTimeStep;
            }

            /** Properties of a CameraQuality. */
            interface ICameraQuality {

                /** CameraQuality name */
                name?: (string|null);

                /** CameraQuality quality */
                quality?: (number|null);
            }

            /** Represents a CameraQuality. */
            class CameraQuality implements ICameraQuality {

                /**
                 * Constructs a new CameraQuality.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.ICameraQuality);

                /** CameraQuality name. */
                public name: string;

                /** CameraQuality quality. */
                public quality: number;

                /**
                 * Encodes the specified CameraQuality message. Does not implicitly {@link message.platform.webots.CameraQuality.verify|verify} messages.
                 * @param message CameraQuality message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.ICameraQuality, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a CameraQuality message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns CameraQuality
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.CameraQuality;
            }

            /** Properties of a CameraExposure. */
            interface ICameraExposure {

                /** CameraExposure name */
                name?: (string|null);

                /** CameraExposure exposure */
                exposure?: (number|null);
            }

            /** Represents a CameraExposure. */
            class CameraExposure implements ICameraExposure {

                /**
                 * Constructs a new CameraExposure.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.ICameraExposure);

                /** CameraExposure name. */
                public name: string;

                /** CameraExposure exposure. */
                public exposure: number;

                /**
                 * Encodes the specified CameraExposure message. Does not implicitly {@link message.platform.webots.CameraExposure.verify|verify} messages.
                 * @param message CameraExposure message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.ICameraExposure, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a CameraExposure message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns CameraExposure
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.CameraExposure;
            }

            /** Properties of an ActuatorRequests. */
            interface IActuatorRequests {

                /** ActuatorRequests motorPositions */
                motorPositions?: (message.platform.webots.IMotorPosition[]|null);

                /** ActuatorRequests motorVelocities */
                motorVelocities?: (message.platform.webots.IMotorVelocity[]|null);

                /** ActuatorRequests motorForces */
                motorForces?: (message.platform.webots.IMotorForce[]|null);

                /** ActuatorRequests motorTorques */
                motorTorques?: (message.platform.webots.IMotorTorque[]|null);

                /** ActuatorRequests motorPids */
                motorPids?: (message.platform.webots.IMotorPID[]|null);

                /** ActuatorRequests sensorTimeSteps */
                sensorTimeSteps?: (message.platform.webots.ISensorTimeStep[]|null);

                /** ActuatorRequests cameraQualities */
                cameraQualities?: (message.platform.webots.ICameraQuality[]|null);

                /** ActuatorRequests cameraExposures */
                cameraExposures?: (message.platform.webots.ICameraExposure[]|null);
            }

            /** Represents an ActuatorRequests. */
            class ActuatorRequests implements IActuatorRequests {

                /**
                 * Constructs a new ActuatorRequests.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.platform.webots.IActuatorRequests);

                /** ActuatorRequests motorPositions. */
                public motorPositions: message.platform.webots.IMotorPosition[];

                /** ActuatorRequests motorVelocities. */
                public motorVelocities: message.platform.webots.IMotorVelocity[];

                /** ActuatorRequests motorForces. */
                public motorForces: message.platform.webots.IMotorForce[];

                /** ActuatorRequests motorTorques. */
                public motorTorques: message.platform.webots.IMotorTorque[];

                /** ActuatorRequests motorPids. */
                public motorPids: message.platform.webots.IMotorPID[];

                /** ActuatorRequests sensorTimeSteps. */
                public sensorTimeSteps: message.platform.webots.ISensorTimeStep[];

                /** ActuatorRequests cameraQualities. */
                public cameraQualities: message.platform.webots.ICameraQuality[];

                /** ActuatorRequests cameraExposures. */
                public cameraExposures: message.platform.webots.ICameraExposure[];

                /**
                 * Encodes the specified ActuatorRequests message. Does not implicitly {@link message.platform.webots.ActuatorRequests.verify|verify} messages.
                 * @param message ActuatorRequests message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.platform.webots.IActuatorRequests, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an ActuatorRequests message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ActuatorRequests
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.platform.webots.ActuatorRequests;
            }
        }
    }

    /** Namespace skill. */
    namespace skill {

        /** Properties of a GetUp. */
        interface IGetUp {
        }

        /** Emit a GetUp Task in order to tell the system to perform the appropriate getup action for it's current situation */
        class GetUp implements IGetUp {

            /**
             * Constructs a new GetUp.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.skill.IGetUp);

            /**
             * Encodes the specified GetUp message. Does not implicitly {@link message.skill.GetUp.verify|verify} messages.
             * @param message GetUp message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.skill.IGetUp, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GetUp message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GetUp
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.skill.GetUp;
        }

        /** Properties of a Kick. */
        interface IKick {

            /** Leg to kick with */
            leg?: (number|null);

            /** The point to kick */
            target?: (Ivec3|null);

            /** corresponding to the force of the kick */
            direction?: (Ivec3|null);
        }

        /** Represents a Kick. */
        class Kick implements IKick {

            /**
             * Constructs a new Kick.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.skill.IKick);

            /** Leg to kick with */
            public leg: number;

            /** The point to kick */
            public target?: (Ivec3|null);

            /** corresponding to the force of the kick */
            public direction?: (Ivec3|null);

            /**
             * Encodes the specified Kick message. Does not implicitly {@link message.skill.Kick.verify|verify} messages.
             * @param message Kick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.skill.IKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Kick message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Kick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.skill.Kick;
        }

        /** Properties of a Look. */
        interface ILook {

            /** Vector from the camera to the point to look at, in torso space */
            rPCt?: (Ivec3|null);

            /** If true, smooth the goal angle using exponential smoothing */
            smooth?: (boolean|null);
        }

        /** A message telling the robot to look in the given direction */
        class Look implements ILook {

            /**
             * Constructs a new Look.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.skill.ILook);

            /** Vector from the camera to the point to look at, in torso space */
            public rPCt?: (Ivec3|null);

            /** If true, smooth the goal angle using exponential smoothing */
            public smooth: boolean;

            /**
             * Encodes the specified Look message. Does not implicitly {@link message.skill.Look.verify|verify} messages.
             * @param message Look message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.skill.ILook, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Look message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Look
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.skill.Look;
        }

        /** Properties of a Walk. */
        interface IWalk {

            /** Walk velocityTarget */
            velocityTarget?: (Ifvec3|null);
        }

        /** Represents a Walk. */
        class Walk implements IWalk {

            /**
             * Constructs a new Walk.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.skill.IWalk);

            /** Walk velocityTarget. */
            public velocityTarget?: (Ifvec3|null);

            /**
             * Encodes the specified Walk message. Does not implicitly {@link message.skill.Walk.verify|verify} messages.
             * @param message Walk message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.skill.IWalk, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Walk message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Walk
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.skill.Walk;
        }
    }

    /** Namespace strategy. */
    namespace strategy {

        /** Properties of a FallRecovery. */
        interface IFallRecovery {
        }

        /**
         * A task that will tell behaviour to manage the falling and getting up state of the system.
         * This includes ensuring the robot will relax its limbs when it is falling, and get up if it has fallen.
         */
        class FallRecovery implements IFallRecovery {

            /**
             * Constructs a new FallRecovery.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.IFallRecovery);

            /**
             * Encodes the specified FallRecovery message. Does not implicitly {@link message.strategy.FallRecovery.verify|verify} messages.
             * @param message FallRecovery message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.IFallRecovery, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FallRecovery message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FallRecovery
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.FallRecovery;
        }

        /** Properties of a FindBall. */
        interface IFindBall {
        }

        /** A Task that requests to handle the finding of the ball when its location is unknown */
        class FindBall implements IFindBall {

            /**
             * Constructs a new FindBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.IFindBall);

            /**
             * Encodes the specified FindBall message. Does not implicitly {@link message.strategy.FindBall.verify|verify} messages.
             * @param message FindBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.IFindBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FindBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FindBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.FindBall;
        }

        /** Properties of a LookAtBall. */
        interface ILookAtBall {
        }

        /** Task requesting to look at the ball */
        class LookAtBall implements ILookAtBall {

            /**
             * Constructs a new LookAtBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.ILookAtBall);

            /**
             * Encodes the specified LookAtBall message. Does not implicitly {@link message.strategy.LookAtBall.verify|verify} messages.
             * @param message LookAtBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.ILookAtBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LookAtBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LookAtBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.LookAtBall;
        }

        /** Properties of a LookAtGoals. */
        interface ILookAtGoals {
        }

        /** Task requesting to look at the goal */
        class LookAtGoals implements ILookAtGoals {

            /**
             * Constructs a new LookAtGoals.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.ILookAtGoals);

            /**
             * Encodes the specified LookAtGoals message. Does not implicitly {@link message.strategy.LookAtGoals.verify|verify} messages.
             * @param message LookAtGoals message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.ILookAtGoals, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a LookAtGoals message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns LookAtGoals
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.LookAtGoals;
        }

        /** Properties of a Ready. */
        interface IReady {

            /** The target position to walk to relative to the field */
            rRFf?: (Ivec3|null);
        }

        /** Ready state strategy for walking onto the field */
        class Ready implements IReady {

            /**
             * Constructs a new Ready.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.IReady);

            /** The target position to walk to relative to the field */
            public rRFf?: (Ivec3|null);

            /**
             * Encodes the specified Ready message. Does not implicitly {@link message.strategy.Ready.verify|verify} messages.
             * @param message Ready message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.IReady, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Ready message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Ready
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.Ready;
        }

        /** Properties of a StandStill. */
        interface IStandStill {
        }

        /** Makes the robot stop walking and stand still */
        class StandStill implements IStandStill {

            /**
             * Constructs a new StandStill.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.IStandStill);

            /**
             * Encodes the specified StandStill message. Does not implicitly {@link message.strategy.StandStill.verify|verify} messages.
             * @param message StandStill message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.IStandStill, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a StandStill message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns StandStill
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.StandStill;
        }

        /** Properties of a WalkToBall. */
        interface IWalkToBall {
        }

        /** Robot walks to the ball or searches if it can't find it */
        class WalkToBall implements IWalkToBall {

            /**
             * Constructs a new WalkToBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.strategy.IWalkToBall);

            /**
             * Encodes the specified WalkToBall message. Does not implicitly {@link message.strategy.WalkToBall.verify|verify} messages.
             * @param message WalkToBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.strategy.IWalkToBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a WalkToBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns WalkToBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.strategy.WalkToBall;
        }
    }

    /** Namespace support. */
    namespace support {

        /** Properties of a FieldDescription. */
        interface IFieldDescription {

            /**
             * @see Law 2
             */
            ballRadius?: (number|null);

            /** Calculated as goal_crossbar_height + goal_crossbar_width */
            goalpostTopHeight?: (number|null);

            /** It's the left one, as if you were looking from halfway */
            goalpostOwnL?: (Ivec2|null);

            /** It's the right one, as if you were looking from halfway */
            goalpostOwnR?: (Ivec2|null);

            /** {x, y} of opposition left goalpost, calculated from goal_width and field_length */
            goalpostOppL?: (Ivec2|null);

            /** {x, y} of opposition right goalpost, calculated from goal_width and field_length */
            goalpostOppR?: (Ivec2|null);

            /** Set of field dimensions loaded from config */
            dimensions?: (message.support.FieldDescription.IFieldDimensions|null);
        }

        /**
         * Information about location and size of field objects and markers.
         * Details about field measurements can be found in 'Law 1 - The Field of Play',
         * in the RoboCup rulebook, here https://humanoid.robocup.org/materials/rules/
         * We have these config values mirrored in `SoccerConfig/data/config/`.
         */
        class FieldDescription implements IFieldDescription {

            /**
             * Constructs a new FieldDescription.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.support.IFieldDescription);

            /**
             * @see Law 2
             */
            public ballRadius: number;

            /** Calculated as goal_crossbar_height + goal_crossbar_width */
            public goalpostTopHeight: number;

            /** It's the left one, as if you were looking from halfway */
            public goalpostOwnL?: (Ivec2|null);

            /** It's the right one, as if you were looking from halfway */
            public goalpostOwnR?: (Ivec2|null);

            /** {x, y} of opposition left goalpost, calculated from goal_width and field_length */
            public goalpostOppL?: (Ivec2|null);

            /** {x, y} of opposition right goalpost, calculated from goal_width and field_length */
            public goalpostOppR?: (Ivec2|null);

            /** Set of field dimensions loaded from config */
            public dimensions?: (message.support.FieldDescription.IFieldDimensions|null);

            /**
             * Encodes the specified FieldDescription message. Does not implicitly {@link message.support.FieldDescription.verify|verify} messages.
             * @param message FieldDescription message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.support.IFieldDescription, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FieldDescription message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FieldDescription
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.FieldDescription;
        }

        namespace FieldDescription {

            /** The cross-sectional shape of the goalposts */
            enum GoalpostType {
                RECTANGLE = 0,
                CIRCLE = 1
            }

            /** Properties of a FieldDimensions. */
            interface IFieldDimensions {

                /** The rules say that the lines must be the same width */
                lineWidth?: (number|null);

                /** The 'touch line' length - AKA the sideline */
                fieldLength?: (number|null);

                /** The 'goal line' length - AKA the baseline */
                fieldWidth?: (number|null);

                /** Cross-sectional shape of goal-posts */
                goalpostType?: (message.support.FieldDescription.GoalpostType|null);

                /** Distance behind the keeper to the net */
                goalDepth?: (number|null);

                /** Distance between inner edges of goal posts */
                goalWidth?: (number|null);

                /** of the field facing goals */
                goalAreaLength?: (number|null);

                /** of the field facing goals */
                goalAreaWidth?: (number|null);

                /** Height to the bottom of the crossbar */
                goalCrossbarHeight?: (number|null);

                /** FieldDimensions goalpostWidth */
                goalpostWidth?: (number|null);

                /** FieldDimensions goalpostDepth */
                goalpostDepth?: (number|null);

                /** FieldDimensions goalCrossbarWidth */
                goalCrossbarWidth?: (number|null);

                /** FieldDimensions goalCrossbarDepth */
                goalCrossbarDepth?: (number|null);

                /** FieldDimensions goalNetHeight */
                goalNetHeight?: (number|null);

                /** Measured from the goal line to the middle of the penalty mark */
                penaltyMarkDistance?: (number|null);

                /** Diameter of the circle in the center of the field */
                centerCircleDiameter?: (number|null);

                /** Minimum width of the border strip around the field */
                borderStripMinWidth?: (number|null);

                /** if you're facing the centre of the field from the goal line */
                penaltyAreaLength?: (number|null);

                /** penalty box facing the centre of the field/goal */
                penaltyAreaWidth?: (number|null);
            }

            /** Measurements of the field which are loaded from config file. In metres */
            class FieldDimensions implements IFieldDimensions {

                /**
                 * Constructs a new FieldDimensions.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.support.FieldDescription.IFieldDimensions);

                /** The rules say that the lines must be the same width */
                public lineWidth: number;

                /** The 'touch line' length - AKA the sideline */
                public fieldLength: number;

                /** The 'goal line' length - AKA the baseline */
                public fieldWidth: number;

                /** Cross-sectional shape of goal-posts */
                public goalpostType: message.support.FieldDescription.GoalpostType;

                /** Distance behind the keeper to the net */
                public goalDepth: number;

                /** Distance between inner edges of goal posts */
                public goalWidth: number;

                /** of the field facing goals */
                public goalAreaLength: number;

                /** of the field facing goals */
                public goalAreaWidth: number;

                /** Height to the bottom of the crossbar */
                public goalCrossbarHeight: number;

                /** FieldDimensions goalpostWidth. */
                public goalpostWidth: number;

                /** FieldDimensions goalpostDepth. */
                public goalpostDepth: number;

                /** FieldDimensions goalCrossbarWidth. */
                public goalCrossbarWidth: number;

                /** FieldDimensions goalCrossbarDepth. */
                public goalCrossbarDepth: number;

                /** FieldDimensions goalNetHeight. */
                public goalNetHeight: number;

                /** Measured from the goal line to the middle of the penalty mark */
                public penaltyMarkDistance: number;

                /** Diameter of the circle in the center of the field */
                public centerCircleDiameter: number;

                /** Minimum width of the border strip around the field */
                public borderStripMinWidth: number;

                /** if you're facing the centre of the field from the goal line */
                public penaltyAreaLength: number;

                /** penalty box facing the centre of the field/goal */
                public penaltyAreaWidth: number;

                /**
                 * Encodes the specified FieldDimensions message. Does not implicitly {@link message.support.FieldDescription.FieldDimensions.verify|verify} messages.
                 * @param message FieldDimensions message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.support.FieldDescription.IFieldDimensions, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a FieldDimensions message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns FieldDimensions
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.FieldDescription.FieldDimensions;
            }
        }

        /** Properties of a GlobalConfig. */
        interface IGlobalConfig {

            /** The robots number in the team. Starts at 1. Should be the same as the robots number (nugus1, nugus2, etc) */
            playerId?: (number|null);

            /** go to teams.cfg, then find <number>=NUbots and the <number> is our team ID. */
            teamId?: (number|null);
        }

        /** Represents a GlobalConfig. */
        class GlobalConfig implements IGlobalConfig {

            /**
             * Constructs a new GlobalConfig.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.support.IGlobalConfig);

            /** The robots number in the team. Starts at 1. Should be the same as the robots number (nugus1, nugus2, etc) */
            public playerId: number;

            /** go to teams.cfg, then find <number>=NUbots and the <number> is our team ID. */
            public teamId: number;

            /**
             * Encodes the specified GlobalConfig message. Does not implicitly {@link message.support.GlobalConfig.verify|verify} messages.
             * @param message GlobalConfig message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.support.IGlobalConfig, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GlobalConfig message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GlobalConfig
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.GlobalConfig;
        }

        /** Namespace nuclear. */
        namespace nuclear {

            /** Properties of a ReactionStatistics. */
            interface IReactionStatistics {

                /** ReactionStatistics name */
                name?: (string|null);

                /** ReactionStatistics triggerName */
                triggerName?: (string|null);

                /** ReactionStatistics functionName */
                functionName?: (string|null);

                /** ReactionStatistics reactionId */
                reactionId?: (number|Long|null);

                /** ReactionStatistics taskId */
                taskId?: (number|Long|null);

                /** ReactionStatistics causeReactionId */
                causeReactionId?: (number|Long|null);

                /** ReactionStatistics causeTaskId */
                causeTaskId?: (number|Long|null);

                /** Timestamp when the message was emitted */
                emitted?: (number|Long|null);

                /** Timestamp when the reaction started */
                started?: (number|Long|null);

                /** Timestamp when the reaction finished */
                finished?: (number|Long|null);
            }

            /** Represents a ReactionStatistics. */
            class ReactionStatistics implements IReactionStatistics {

                /**
                 * Constructs a new ReactionStatistics.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.support.nuclear.IReactionStatistics);

                /** ReactionStatistics name. */
                public name: string;

                /** ReactionStatistics triggerName. */
                public triggerName: string;

                /** ReactionStatistics functionName. */
                public functionName: string;

                /** ReactionStatistics reactionId. */
                public reactionId: (number|Long);

                /** ReactionStatistics taskId. */
                public taskId: (number|Long);

                /** ReactionStatistics causeReactionId. */
                public causeReactionId: (number|Long);

                /** ReactionStatistics causeTaskId. */
                public causeTaskId: (number|Long);

                /** Timestamp when the message was emitted */
                public emitted: (number|Long);

                /** Timestamp when the reaction started */
                public started: (number|Long);

                /** Timestamp when the reaction finished */
                public finished: (number|Long);

                /**
                 * Encodes the specified ReactionStatistics message. Does not implicitly {@link message.support.nuclear.ReactionStatistics.verify|verify} messages.
                 * @param message ReactionStatistics message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.support.nuclear.IReactionStatistics, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a ReactionStatistics message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ReactionStatistics
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.nuclear.ReactionStatistics;
            }
        }

        /** Namespace nusight. */
        namespace nusight {

            /** Properties of a Command. */
            interface ICommand {

                /** Command command */
                command?: (string|null);
            }

            /** Represents a Command. */
            class Command implements ICommand {

                /**
                 * Constructs a new Command.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.support.nusight.ICommand);

                /** Command command. */
                public command: string;

                /**
                 * Encodes the specified Command message. Does not implicitly {@link message.support.nusight.Command.verify|verify} messages.
                 * @param message Command message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.support.nusight.ICommand, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Command message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Command
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.nusight.Command;
            }

            /** Properties of a DataPoint. */
            interface IDataPoint {

                /** DataPoint label */
                label?: (string|null);

                /** DataPoint value */
                value?: (number[]|null);

                /** DataPoint timestamp */
                timestamp?: (google.protobuf.ITimestamp|null);
            }

            /** Represents a DataPoint. */
            class DataPoint implements IDataPoint {

                /**
                 * Constructs a new DataPoint.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.support.nusight.IDataPoint);

                /** DataPoint label. */
                public label: string;

                /** DataPoint value. */
                public value: number[];

                /** DataPoint timestamp. */
                public timestamp?: (google.protobuf.ITimestamp|null);

                /**
                 * Encodes the specified DataPoint message. Does not implicitly {@link message.support.nusight.DataPoint.verify|verify} messages.
                 * @param message DataPoint message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.support.nusight.IDataPoint, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a DataPoint message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns DataPoint
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.nusight.DataPoint;
            }

            /** Properties of an Overview. */
            interface IOverview {

                /** The timestamp this overview packet was sent */
                timestamp?: (google.protobuf.ITimestamp|null);

                /** Overview robotId */
                robotId?: (number|null);

                /** The name of the binary this robot is executing */
                roleName?: (string|null);

                /**
                 * @see message::input::Sensors::battery
                 */
                battery?: (number|null);

                /**
                 * @see message::input::Sensors::voltage
                 */
                voltage?: (number|null);

                /** The current behaviour state. List of possible states in message::behaviour::Behaviour */
                behaviourState?: (message.behaviour.Behaviour.State|null);

                /** Localisation measured in field space */
                robotPosition?: (Ifvec3|null);

                /** Overview robotPositionCovariance */
                robotPositionCovariance?: (Ifmat3|null);

                /** The ball position measured in field space */
                ballPosition?: (Ifvec2|null);

                /** Overview ballPositionCovariance */
                ballPositionCovariance?: (Ifmat2|null);

                /** The {x, y} location we are kicking to */
                kickTarget?: (Ifvec2|null);

                /** The type of game being played. From Game Controller */
                gameMode?: (message.input.GameState.Data.Mode|null);

                /** The current stage of gameplay. For Game Controller */
                gamePhase?: (message.input.GameState.Data.Phase|null);

                /** Why the penalty was given. For Game Controller */
                penaltyReason?: (message.input.GameState.Data.PenaltyReason|null);

                /** Last time the camera image was seen */
                lastCameraImage?: (google.protobuf.ITimestamp|null);

                /** Last time vision objects were seen */
                lastSeenBall?: (google.protobuf.ITimestamp|null);

                /** Overview lastSeenGoal */
                lastSeenGoal?: (google.protobuf.ITimestamp|null);

                /** required rotational velocity about the z axis. In torso space */
                walkCommand?: (Ifvec3|null);
            }

            /** Represents an Overview. */
            class Overview implements IOverview {

                /**
                 * Constructs a new Overview.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.support.nusight.IOverview);

                /** The timestamp this overview packet was sent */
                public timestamp?: (google.protobuf.ITimestamp|null);

                /** Overview robotId. */
                public robotId: number;

                /** The name of the binary this robot is executing */
                public roleName: string;

                /**
                 * @see message::input::Sensors::battery
                 */
                public battery: number;

                /**
                 * @see message::input::Sensors::voltage
                 */
                public voltage: number;

                /** The current behaviour state. List of possible states in message::behaviour::Behaviour */
                public behaviourState: message.behaviour.Behaviour.State;

                /** Localisation measured in field space */
                public robotPosition?: (Ifvec3|null);

                /** Overview robotPositionCovariance. */
                public robotPositionCovariance?: (Ifmat3|null);

                /** The ball position measured in field space */
                public ballPosition?: (Ifvec2|null);

                /** Overview ballPositionCovariance. */
                public ballPositionCovariance?: (Ifmat2|null);

                /** The {x, y} location we are kicking to */
                public kickTarget?: (Ifvec2|null);

                /** The type of game being played. From Game Controller */
                public gameMode: message.input.GameState.Data.Mode;

                /** The current stage of gameplay. For Game Controller */
                public gamePhase: message.input.GameState.Data.Phase;

                /** Why the penalty was given. For Game Controller */
                public penaltyReason: message.input.GameState.Data.PenaltyReason;

                /** Last time the camera image was seen */
                public lastCameraImage?: (google.protobuf.ITimestamp|null);

                /** Last time vision objects were seen */
                public lastSeenBall?: (google.protobuf.ITimestamp|null);

                /** Overview lastSeenGoal. */
                public lastSeenGoal?: (google.protobuf.ITimestamp|null);

                /** required rotational velocity about the z axis. In torso space */
                public walkCommand?: (Ifvec3|null);

                /**
                 * Encodes the specified Overview message. Does not implicitly {@link message.support.nusight.Overview.verify|verify} messages.
                 * @param message Overview message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.support.nusight.IOverview, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an Overview message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Overview
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.support.nusight.Overview;
            }
        }
    }

    /** Namespace vision. */
    namespace vision {

        /** Properties of a Ball. */
        interface IBall {

            /** Central axis unit vector of the ball from the camera in camera space */
            uBCc?: (Ifvec3|null);

            /** Cosine of the angle from the centre of the ball to the edge of the ball */
            radius?: (number|null);

            /** Different methods of measurements are taken to help ball localisation determine the true position */
            measurements?: (message.vision.Ball.IMeasurement[]|null);

            /** the reason it was discarded */
            colour?: (Ifvec4|null);
        }

        /** Represents a Ball. */
        class Ball implements IBall {

            /**
             * Constructs a new Ball.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IBall);

            /** Central axis unit vector of the ball from the camera in camera space */
            public uBCc?: (Ifvec3|null);

            /** Cosine of the angle from the centre of the ball to the edge of the ball */
            public radius: number;

            /** Different methods of measurements are taken to help ball localisation determine the true position */
            public measurements: message.vision.Ball.IMeasurement[];

            /** the reason it was discarded */
            public colour?: (Ifvec4|null);

            /**
             * Encodes the specified Ball message. Does not implicitly {@link message.vision.Ball.verify|verify} messages.
             * @param message Ball message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Ball message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Ball
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Ball;
        }

        namespace Ball {

            /** Method of measuring the ball position */
            enum MeasurementType {
                UNKNOWN = 0,
                PROJECTION = 1,
                ANGULAR = 2
            }

            /** Properties of a Measurement. */
            interface IMeasurement {

                /** Measurement type */
                type?: (message.vision.Ball.MeasurementType|null);

                /** Gives a better representation of the error of the measurement, since error comes from the angular position */
                srBCc?: (Ifvec3|null);

                /** Confidence of this measurement, for localisation */
                covariance?: (Ifmat3|null);
            }

            /** and the certainty of the measurement */
            class Measurement implements IMeasurement {

                /**
                 * Constructs a new Measurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.vision.Ball.IMeasurement);

                /** Measurement type. */
                public type: message.vision.Ball.MeasurementType;

                /** Gives a better representation of the error of the measurement, since error comes from the angular position */
                public srBCc?: (Ifvec3|null);

                /** Confidence of this measurement, for localisation */
                public covariance?: (Ifmat3|null);

                /**
                 * Encodes the specified Measurement message. Does not implicitly {@link message.vision.Ball.Measurement.verify|verify} messages.
                 * @param message Measurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.vision.Ball.IMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Measurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Measurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Ball.Measurement;
            }
        }

        /** Properties of a Balls. */
        interface IBalls {

            /** ID of the camera that took the image containing the balls */
            id?: (number|null);

            /** When the image was taken that this measurement is derived from */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            Hcw?: (Imat4|null);

            /** Could be empty, if there were no balls detected in the image */
            balls?: (message.vision.IBall[]|null);
        }

        /** Represents a Balls. */
        class Balls implements IBalls {

            /**
             * Constructs a new Balls.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IBalls);

            /** ID of the camera that took the image containing the balls */
            public id: number;

            /** When the image was taken that this measurement is derived from */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            public Hcw?: (Imat4|null);

            /** Could be empty, if there were no balls detected in the image */
            public balls: message.vision.IBall[];

            /**
             * Encodes the specified Balls message. Does not implicitly {@link message.vision.Balls.verify|verify} messages.
             * @param message Balls message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IBalls, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Balls message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Balls
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Balls;
        }

        /** Properties of a FieldLines. */
        interface IFieldLines {

            /** ID of the camera that took the image containing the lines */
            id?: (number|null);

            /** When the image was taken that this measurement is derived from */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** Affine3d transform from world space to camera space at the time that the image was taken */
            Hcw?: (Imat4|null);

            /** Could be empty, if there were no lines detected in the image */
            points?: (Ifvec3[]|null);
        }

        /** Represents a FieldLines. */
        class FieldLines implements IFieldLines {

            /**
             * Constructs a new FieldLines.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IFieldLines);

            /** ID of the camera that took the image containing the lines */
            public id: number;

            /** When the image was taken that this measurement is derived from */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** Affine3d transform from world space to camera space at the time that the image was taken */
            public Hcw?: (Imat4|null);

            /** Could be empty, if there were no lines detected in the image */
            public points: Ifvec3[];

            /**
             * Encodes the specified FieldLines message. Does not implicitly {@link message.vision.FieldLines.verify|verify} messages.
             * @param message FieldLines message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IFieldLines, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FieldLines message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FieldLines
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.FieldLines;
        }

        /** Properties of a Goal. */
        interface IGoal {

            /** Goal side */
            side?: (message.vision.Goal.Side|null);

            /** Goal post */
            post?: (message.vision.Goal.IPost|null);

            /** Goal measurements */
            measurements?: (message.vision.Goal.IMeasurement[]|null);

            /** Goal screenAngular */
            screenAngular?: (Ifvec2|null);

            /** Goal angularSize */
            angularSize?: (Ifvec2|null);
        }

        /** Represents a Goal. */
        class Goal implements IGoal {

            /**
             * Constructs a new Goal.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IGoal);

            /** Goal side. */
            public side: message.vision.Goal.Side;

            /** Goal post. */
            public post?: (message.vision.Goal.IPost|null);

            /** Goal measurements. */
            public measurements: message.vision.Goal.IMeasurement[];

            /** Goal screenAngular. */
            public screenAngular?: (Ifvec2|null);

            /** Goal angularSize. */
            public angularSize?: (Ifvec2|null);

            /**
             * Encodes the specified Goal message. Does not implicitly {@link message.vision.Goal.verify|verify} messages.
             * @param message Goal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Goal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Goal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Goal;
        }

        namespace Goal {

            /** Side enum. */
            enum Side {
                UNKNOWN_SIDE = 0,
                LEFT = 1,
                RIGHT = 2
            }

            /** Which team the goals belong to */
            enum Team {
                UNKNOWN_TEAM = 0,
                OWN = 1,
                OPPONENT = 2
            }

            /** MeasurementType enum. */
            enum MeasurementType {
                UNKNOWN = 0,
                CENTRE = 1
            }

            /** Properties of a Measurement. */
            interface IMeasurement {

                /** Measurement type */
                type?: (message.vision.Goal.MeasurementType|null);

                /** Normal vectors point inwards towards the centre of the object */
                srGCc?: (Ifvec3|null);

                /** Measurement covariance */
                covariance?: (Ifmat3|null);
            }

            /** Represents a Measurement. */
            class Measurement implements IMeasurement {

                /**
                 * Constructs a new Measurement.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.vision.Goal.IMeasurement);

                /** Measurement type. */
                public type: message.vision.Goal.MeasurementType;

                /** Normal vectors point inwards towards the centre of the object */
                public srGCc?: (Ifvec3|null);

                /** Measurement covariance. */
                public covariance?: (Ifmat3|null);

                /**
                 * Encodes the specified Measurement message. Does not implicitly {@link message.vision.Goal.Measurement.verify|verify} messages.
                 * @param message Measurement message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.vision.Goal.IMeasurement, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Measurement message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Measurement
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Goal.Measurement;
            }

            /** Properties of a Post. */
            interface IPost {

                /** Vector pointing to top point of post in camera space */
                top?: (Ifvec3|null);

                /** Vector pointing to bottom point of post in camera space */
                bottom?: (Ifvec3|null);

                /** Distance from camera to goals, in metres */
                distance?: (number|null);
            }

            /** Represents a Post. */
            class Post implements IPost {

                /**
                 * Constructs a new Post.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: message.vision.Goal.IPost);

                /** Vector pointing to top point of post in camera space */
                public top?: (Ifvec3|null);

                /** Vector pointing to bottom point of post in camera space */
                public bottom?: (Ifvec3|null);

                /** Distance from camera to goals, in metres */
                public distance: number;

                /**
                 * Encodes the specified Post message. Does not implicitly {@link message.vision.Goal.Post.verify|verify} messages.
                 * @param message Post message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: message.vision.Goal.IPost, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Post message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Post
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Goal.Post;
            }
        }

        /** Properties of a Goals. */
        interface IGoals {

            /** Goals id */
            id?: (number|null);

            /** Goals timestamp */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            Hcw?: (Imat4|null);

            /** Goals goals */
            goals?: (message.vision.IGoal[]|null);
        }

        /** Represents a Goals. */
        class Goals implements IGoals {

            /**
             * Constructs a new Goals.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IGoals);

            /** Goals id. */
            public id: number;

            /** Goals timestamp. */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            public Hcw?: (Imat4|null);

            /** Goals goals. */
            public goals: message.vision.IGoal[];

            /**
             * Encodes the specified Goals message. Does not implicitly {@link message.vision.Goals.verify|verify} messages.
             * @param message Goals message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IGoals, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Goals message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Goals
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Goals;
        }

        /** Properties of a GreenHorizon. */
        interface IGreenHorizon {

            /** A set of rays in world space, projected to the ground. They comprise the points in the convex hull */
            horizon?: (Ifvec3[]|null);

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            Hcw?: (Imat4|null);

            /** ID of the camera that took the image containing the green horizon */
            id?: (number|null);

            /** The visual mesh that this green horizon was calculated from */
            mesh?: (message.vision.IVisualMesh|null);

            /** When the image was taken that this measurement is derived from */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** A mapping from class names to column indices (for accessing the corresponding classification column) */
            classMap?: ({ [k: string]: number }|null);

            /** Ground truth data from webots */
            visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);
        }

        /** Represents a GreenHorizon. */
        class GreenHorizon implements IGreenHorizon {

            /**
             * Constructs a new GreenHorizon.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IGreenHorizon);

            /** A set of rays in world space, projected to the ground. They comprise the points in the convex hull */
            public horizon: Ifvec3[];

            /** Isometry3d transform from world space to camera space at the time that the image was taken */
            public Hcw?: (Imat4|null);

            /** ID of the camera that took the image containing the green horizon */
            public id: number;

            /** The visual mesh that this green horizon was calculated from */
            public mesh?: (message.vision.IVisualMesh|null);

            /** When the image was taken that this measurement is derived from */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** A mapping from class names to column indices (for accessing the corresponding classification column) */
            public classMap: { [k: string]: number };

            /** Ground truth data from webots */
            public visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);

            /**
             * Encodes the specified GreenHorizon message. Does not implicitly {@link message.vision.GreenHorizon.verify|verify} messages.
             * @param message GreenHorizon message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IGreenHorizon, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GreenHorizon message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GreenHorizon
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.GreenHorizon;
        }

        /** Properties of an Obstacle. */
        interface IObstacle {

            /** ID of the camera that took the image containing the obstacle */
            id?: (number|null);

            /** When the obstacle was seen */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** {x, y} points bounding the location of the obstacle */
            shape?: (message.IPolygon|null);

            /** Obstacle team */
            team?: (message.vision.Obstacle.Team|null);
        }

        /** Represents an Obstacle. */
        class Obstacle implements IObstacle {

            /**
             * Constructs a new Obstacle.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IObstacle);

            /** ID of the camera that took the image containing the obstacle */
            public id: number;

            /** When the obstacle was seen */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** {x, y} points bounding the location of the obstacle */
            public shape?: (message.IPolygon|null);

            /** Obstacle team. */
            public team: message.vision.Obstacle.Team;

            /**
             * Encodes the specified Obstacle message. Does not implicitly {@link message.vision.Obstacle.verify|verify} messages.
             * @param message Obstacle message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IObstacle, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an Obstacle message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Obstacle
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Obstacle;
        }

        namespace Obstacle {

            /** Team enum. */
            enum Team {
                UNKNOWN_TEAM = 0,
                MAGENTA = 1,
                CYAN = 2
            }
        }

        /** Properties of an Obstacles. */
        interface IObstacles {

            /** Obstacles lines */
            lines?: (message.vision.IObstacle[]|null);
        }

        /** Represents an Obstacles. */
        class Obstacles implements IObstacles {

            /**
             * Constructs a new Obstacles.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IObstacles);

            /** Obstacles lines. */
            public lines: message.vision.IObstacle[];

            /**
             * Encodes the specified Obstacles message. Does not implicitly {@link message.vision.Obstacles.verify|verify} messages.
             * @param message Obstacles message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IObstacles, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an Obstacles message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Obstacles
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.Obstacles;
        }

        /** Properties of a VisualMesh. */
        interface IVisualMesh {

            /** Time this message was created */
            timestamp?: (google.protobuf.ITimestamp|null);

            /** ID of the camera that this mesh was constructed from */
            id?: (number|null);

            /** name of the camera that this mesh was constructed from */
            name?: (string|null);

            /** World to camera transform that this mesh was constructed with */
            Hcw?: (Imat4|null);

            /** Pixel coordinates projected from the rays */
            coordinates?: (Ifmat|null);

            /** Neighbours of each node in the mesh. Each row corresponds to a node and the columns are the neighbours */
            neighbourhood?: (Iimat|null);

            /** The corresponding global indices for each on-screen node */
            indices?: (number[]|null);

            /** class. The mapping from columns to classes is dependant on how the classes were presented during training */
            classifications?: (Ifmat|null);

            /** Unit vectors pointing from the camera to each node in the mesh */
            rays?: (Ifmat|null);

            /** A mapping from class names to column indices (for accessing the corresponding classification column) */
            classMap?: ({ [k: string]: number }|null);

            /** Ground truth data from webots */
            visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);
        }

        /** Represents a VisualMesh. */
        class VisualMesh implements IVisualMesh {

            /**
             * Constructs a new VisualMesh.
             * @param [properties] Properties to set
             */
            constructor(properties?: message.vision.IVisualMesh);

            /** Time this message was created */
            public timestamp?: (google.protobuf.ITimestamp|null);

            /** ID of the camera that this mesh was constructed from */
            public id: number;

            /** name of the camera that this mesh was constructed from */
            public name: string;

            /** World to camera transform that this mesh was constructed with */
            public Hcw?: (Imat4|null);

            /** Pixel coordinates projected from the rays */
            public coordinates?: (Ifmat|null);

            /** Neighbours of each node in the mesh. Each row corresponds to a node and the columns are the neighbours */
            public neighbourhood?: (Iimat|null);

            /** The corresponding global indices for each on-screen node */
            public indices: number[];

            /** class. The mapping from columns to classes is dependant on how the classes were presented during training */
            public classifications?: (Ifmat|null);

            /** Unit vectors pointing from the camera to each node in the mesh */
            public rays?: (Ifmat|null);

            /** A mapping from class names to column indices (for accessing the corresponding classification column) */
            public classMap: { [k: string]: number };

            /** Ground truth data from webots */
            public visionGroundTruth?: (message.platform.webots.IVisionGroundTruth|null);

            /**
             * Encodes the specified VisualMesh message. Does not implicitly {@link message.vision.VisualMesh.verify|verify} messages.
             * @param message VisualMesh message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: message.vision.IVisualMesh, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a VisualMesh message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns VisualMesh
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): message.vision.VisualMesh;
        }
    }
}

/** Properties of a vec. */
export interface Ivec {

    /** vec v */
    v?: (number[]|null);
}

/** Represents a vec. */
export class vec implements Ivec {

    /**
     * Constructs a new vec.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec);

    /** vec v. */
    public v: number[];

    /**
     * Encodes the specified vec message. Does not implicitly {@link vec.verify|verify} messages.
     * @param message vec message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec;
}

/** Properties of a fvec. */
export interface Ifvec {

    /** fvec v */
    v?: (number[]|null);
}

/** Represents a fvec. */
export class fvec implements Ifvec {

    /**
     * Constructs a new fvec.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec);

    /** fvec v. */
    public v: number[];

    /**
     * Encodes the specified fvec message. Does not implicitly {@link fvec.verify|verify} messages.
     * @param message fvec message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec;
}

/** Properties of an ivec. */
export interface Iivec {

    /** ivec v */
    v?: (number[]|null);
}

/** Represents an ivec. */
export class ivec implements Iivec {

    /**
     * Constructs a new ivec.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec);

    /** ivec v. */
    public v: number[];

    /**
     * Encodes the specified ivec message. Does not implicitly {@link ivec.verify|verify} messages.
     * @param message ivec message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec;
}

/** Properties of an uvec. */
export interface Iuvec {

    /** uvec v */
    v?: (number[]|null);
}

/** Represents an uvec. */
export class uvec implements Iuvec {

    /**
     * Constructs a new uvec.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec);

    /** uvec v. */
    public v: number[];

    /**
     * Encodes the specified uvec message. Does not implicitly {@link uvec.verify|verify} messages.
     * @param message uvec message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec;
}

/** Properties of a cvec. */
export interface Icvec {

    /** cvec v */
    v?: (Uint8Array|null);
}

/** Represents a cvec. */
export class cvec implements Icvec {

    /**
     * Constructs a new cvec.
     * @param [properties] Properties to set
     */
    constructor(properties?: Icvec);

    /** cvec v. */
    public v: Uint8Array;

    /**
     * Encodes the specified cvec message. Does not implicitly {@link cvec.verify|verify} messages.
     * @param message cvec message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Icvec, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a cvec message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns cvec
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): cvec;
}

/** Properties of a vec2. */
export interface Ivec2 {

    /** vec2 x */
    x?: (number|null);

    /** vec2 y */
    y?: (number|null);
}

/** Represents a vec2. */
export class vec2 implements Ivec2 {

    /**
     * Constructs a new vec2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec2);

    /** vec2 x. */
    public x: number;

    /** vec2 y. */
    public y: number;

    /**
     * Encodes the specified vec2 message. Does not implicitly {@link vec2.verify|verify} messages.
     * @param message vec2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec2;
}

/** Properties of a fvec2. */
export interface Ifvec2 {

    /** fvec2 x */
    x?: (number|null);

    /** fvec2 y */
    y?: (number|null);
}

/** Represents a fvec2. */
export class fvec2 implements Ifvec2 {

    /**
     * Constructs a new fvec2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec2);

    /** fvec2 x. */
    public x: number;

    /** fvec2 y. */
    public y: number;

    /**
     * Encodes the specified fvec2 message. Does not implicitly {@link fvec2.verify|verify} messages.
     * @param message fvec2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec2;
}

/** Properties of an ivec2. */
export interface Iivec2 {

    /** ivec2 x */
    x?: (number|null);

    /** ivec2 y */
    y?: (number|null);
}

/** Represents an ivec2. */
export class ivec2 implements Iivec2 {

    /**
     * Constructs a new ivec2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec2);

    /** ivec2 x. */
    public x: number;

    /** ivec2 y. */
    public y: number;

    /**
     * Encodes the specified ivec2 message. Does not implicitly {@link ivec2.verify|verify} messages.
     * @param message ivec2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec2;
}

/** Properties of an uvec2. */
export interface Iuvec2 {

    /** uvec2 x */
    x?: (number|null);

    /** uvec2 y */
    y?: (number|null);
}

/** Represents an uvec2. */
export class uvec2 implements Iuvec2 {

    /**
     * Constructs a new uvec2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec2);

    /** uvec2 x. */
    public x: number;

    /** uvec2 y. */
    public y: number;

    /**
     * Encodes the specified uvec2 message. Does not implicitly {@link uvec2.verify|verify} messages.
     * @param message uvec2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec2;
}

/** Properties of a vec3. */
export interface Ivec3 {

    /** vec3 x */
    x?: (number|null);

    /** vec3 y */
    y?: (number|null);

    /** vec3 z */
    z?: (number|null);
}

/** Represents a vec3. */
export class vec3 implements Ivec3 {

    /**
     * Constructs a new vec3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec3);

    /** vec3 x. */
    public x: number;

    /** vec3 y. */
    public y: number;

    /** vec3 z. */
    public z: number;

    /**
     * Encodes the specified vec3 message. Does not implicitly {@link vec3.verify|verify} messages.
     * @param message vec3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec3;
}

/** Properties of a fvec3. */
export interface Ifvec3 {

    /** fvec3 x */
    x?: (number|null);

    /** fvec3 y */
    y?: (number|null);

    /** fvec3 z */
    z?: (number|null);
}

/** Represents a fvec3. */
export class fvec3 implements Ifvec3 {

    /**
     * Constructs a new fvec3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec3);

    /** fvec3 x. */
    public x: number;

    /** fvec3 y. */
    public y: number;

    /** fvec3 z. */
    public z: number;

    /**
     * Encodes the specified fvec3 message. Does not implicitly {@link fvec3.verify|verify} messages.
     * @param message fvec3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec3;
}

/** Properties of an ivec3. */
export interface Iivec3 {

    /** ivec3 x */
    x?: (number|null);

    /** ivec3 y */
    y?: (number|null);

    /** ivec3 z */
    z?: (number|null);
}

/** Represents an ivec3. */
export class ivec3 implements Iivec3 {

    /**
     * Constructs a new ivec3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec3);

    /** ivec3 x. */
    public x: number;

    /** ivec3 y. */
    public y: number;

    /** ivec3 z. */
    public z: number;

    /**
     * Encodes the specified ivec3 message. Does not implicitly {@link ivec3.verify|verify} messages.
     * @param message ivec3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec3;
}

/** Properties of an uvec3. */
export interface Iuvec3 {

    /** uvec3 x */
    x?: (number|null);

    /** uvec3 y */
    y?: (number|null);

    /** uvec3 z */
    z?: (number|null);
}

/** Represents an uvec3. */
export class uvec3 implements Iuvec3 {

    /**
     * Constructs a new uvec3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec3);

    /** uvec3 x. */
    public x: number;

    /** uvec3 y. */
    public y: number;

    /** uvec3 z. */
    public z: number;

    /**
     * Encodes the specified uvec3 message. Does not implicitly {@link uvec3.verify|verify} messages.
     * @param message uvec3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec3;
}

/** Properties of a vec4. */
export interface Ivec4 {

    /** vec4 x */
    x?: (number|null);

    /** vec4 y */
    y?: (number|null);

    /** vec4 z */
    z?: (number|null);

    /** vec4 t */
    t?: (number|null);
}

/** Represents a vec4. */
export class vec4 implements Ivec4 {

    /**
     * Constructs a new vec4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec4);

    /** vec4 x. */
    public x: number;

    /** vec4 y. */
    public y: number;

    /** vec4 z. */
    public z: number;

    /** vec4 t. */
    public t: number;

    /**
     * Encodes the specified vec4 message. Does not implicitly {@link vec4.verify|verify} messages.
     * @param message vec4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec4;
}

/** Properties of a fvec4. */
export interface Ifvec4 {

    /** fvec4 x */
    x?: (number|null);

    /** fvec4 y */
    y?: (number|null);

    /** fvec4 z */
    z?: (number|null);

    /** fvec4 t */
    t?: (number|null);
}

/** Represents a fvec4. */
export class fvec4 implements Ifvec4 {

    /**
     * Constructs a new fvec4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec4);

    /** fvec4 x. */
    public x: number;

    /** fvec4 y. */
    public y: number;

    /** fvec4 z. */
    public z: number;

    /** fvec4 t. */
    public t: number;

    /**
     * Encodes the specified fvec4 message. Does not implicitly {@link fvec4.verify|verify} messages.
     * @param message fvec4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec4;
}

/** Properties of an ivec4. */
export interface Iivec4 {

    /** ivec4 x */
    x?: (number|null);

    /** ivec4 y */
    y?: (number|null);

    /** ivec4 z */
    z?: (number|null);

    /** ivec4 t */
    t?: (number|null);
}

/** Represents an ivec4. */
export class ivec4 implements Iivec4 {

    /**
     * Constructs a new ivec4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec4);

    /** ivec4 x. */
    public x: number;

    /** ivec4 y. */
    public y: number;

    /** ivec4 z. */
    public z: number;

    /** ivec4 t. */
    public t: number;

    /**
     * Encodes the specified ivec4 message. Does not implicitly {@link ivec4.verify|verify} messages.
     * @param message ivec4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec4;
}

/** Properties of an uvec4. */
export interface Iuvec4 {

    /** uvec4 x */
    x?: (number|null);

    /** uvec4 y */
    y?: (number|null);

    /** uvec4 z */
    z?: (number|null);

    /** uvec4 t */
    t?: (number|null);
}

/** Represents an uvec4. */
export class uvec4 implements Iuvec4 {

    /**
     * Constructs a new uvec4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec4);

    /** uvec4 x. */
    public x: number;

    /** uvec4 y. */
    public y: number;

    /** uvec4 z. */
    public z: number;

    /** uvec4 t. */
    public t: number;

    /**
     * Encodes the specified uvec4 message. Does not implicitly {@link uvec4.verify|verify} messages.
     * @param message uvec4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec4;
}

/** Properties of a vec5. */
export interface Ivec5 {

    /** vec5 s0 */
    s0?: (number|null);

    /** vec5 s1 */
    s1?: (number|null);

    /** vec5 s2 */
    s2?: (number|null);

    /** vec5 s3 */
    s3?: (number|null);

    /** vec5 s4 */
    s4?: (number|null);
}

/** Represents a vec5. */
export class vec5 implements Ivec5 {

    /**
     * Constructs a new vec5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec5);

    /** vec5 s0. */
    public s0: number;

    /** vec5 s1. */
    public s1: number;

    /** vec5 s2. */
    public s2: number;

    /** vec5 s3. */
    public s3: number;

    /** vec5 s4. */
    public s4: number;

    /**
     * Encodes the specified vec5 message. Does not implicitly {@link vec5.verify|verify} messages.
     * @param message vec5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec5;
}

/** Properties of a fvec5. */
export interface Ifvec5 {

    /** fvec5 s0 */
    s0?: (number|null);

    /** fvec5 s1 */
    s1?: (number|null);

    /** fvec5 s2 */
    s2?: (number|null);

    /** fvec5 s3 */
    s3?: (number|null);

    /** fvec5 s4 */
    s4?: (number|null);
}

/** Represents a fvec5. */
export class fvec5 implements Ifvec5 {

    /**
     * Constructs a new fvec5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec5);

    /** fvec5 s0. */
    public s0: number;

    /** fvec5 s1. */
    public s1: number;

    /** fvec5 s2. */
    public s2: number;

    /** fvec5 s3. */
    public s3: number;

    /** fvec5 s4. */
    public s4: number;

    /**
     * Encodes the specified fvec5 message. Does not implicitly {@link fvec5.verify|verify} messages.
     * @param message fvec5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec5;
}

/** Properties of an ivec5. */
export interface Iivec5 {

    /** ivec5 s0 */
    s0?: (number|null);

    /** ivec5 s1 */
    s1?: (number|null);

    /** ivec5 s2 */
    s2?: (number|null);

    /** ivec5 s3 */
    s3?: (number|null);

    /** ivec5 s4 */
    s4?: (number|null);
}

/** Represents an ivec5. */
export class ivec5 implements Iivec5 {

    /**
     * Constructs a new ivec5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec5);

    /** ivec5 s0. */
    public s0: number;

    /** ivec5 s1. */
    public s1: number;

    /** ivec5 s2. */
    public s2: number;

    /** ivec5 s3. */
    public s3: number;

    /** ivec5 s4. */
    public s4: number;

    /**
     * Encodes the specified ivec5 message. Does not implicitly {@link ivec5.verify|verify} messages.
     * @param message ivec5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec5;
}

/** Properties of an uvec5. */
export interface Iuvec5 {

    /** uvec5 s0 */
    s0?: (number|null);

    /** uvec5 s1 */
    s1?: (number|null);

    /** uvec5 s2 */
    s2?: (number|null);

    /** uvec5 s3 */
    s3?: (number|null);

    /** uvec5 s4 */
    s4?: (number|null);
}

/** Represents an uvec5. */
export class uvec5 implements Iuvec5 {

    /**
     * Constructs a new uvec5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec5);

    /** uvec5 s0. */
    public s0: number;

    /** uvec5 s1. */
    public s1: number;

    /** uvec5 s2. */
    public s2: number;

    /** uvec5 s3. */
    public s3: number;

    /** uvec5 s4. */
    public s4: number;

    /**
     * Encodes the specified uvec5 message. Does not implicitly {@link uvec5.verify|verify} messages.
     * @param message uvec5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec5;
}

/** Properties of a vec6. */
export interface Ivec6 {

    /** vec6 s0 */
    s0?: (number|null);

    /** vec6 s1 */
    s1?: (number|null);

    /** vec6 s2 */
    s2?: (number|null);

    /** vec6 s3 */
    s3?: (number|null);

    /** vec6 s4 */
    s4?: (number|null);

    /** vec6 s5 */
    s5?: (number|null);
}

/** Represents a vec6. */
export class vec6 implements Ivec6 {

    /**
     * Constructs a new vec6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec6);

    /** vec6 s0. */
    public s0: number;

    /** vec6 s1. */
    public s1: number;

    /** vec6 s2. */
    public s2: number;

    /** vec6 s3. */
    public s3: number;

    /** vec6 s4. */
    public s4: number;

    /** vec6 s5. */
    public s5: number;

    /**
     * Encodes the specified vec6 message. Does not implicitly {@link vec6.verify|verify} messages.
     * @param message vec6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec6;
}

/** Properties of a fvec6. */
export interface Ifvec6 {

    /** fvec6 s0 */
    s0?: (number|null);

    /** fvec6 s1 */
    s1?: (number|null);

    /** fvec6 s2 */
    s2?: (number|null);

    /** fvec6 s3 */
    s3?: (number|null);

    /** fvec6 s4 */
    s4?: (number|null);

    /** fvec6 s5 */
    s5?: (number|null);
}

/** Represents a fvec6. */
export class fvec6 implements Ifvec6 {

    /**
     * Constructs a new fvec6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec6);

    /** fvec6 s0. */
    public s0: number;

    /** fvec6 s1. */
    public s1: number;

    /** fvec6 s2. */
    public s2: number;

    /** fvec6 s3. */
    public s3: number;

    /** fvec6 s4. */
    public s4: number;

    /** fvec6 s5. */
    public s5: number;

    /**
     * Encodes the specified fvec6 message. Does not implicitly {@link fvec6.verify|verify} messages.
     * @param message fvec6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec6;
}

/** Properties of an ivec6. */
export interface Iivec6 {

    /** ivec6 s0 */
    s0?: (number|null);

    /** ivec6 s1 */
    s1?: (number|null);

    /** ivec6 s2 */
    s2?: (number|null);

    /** ivec6 s3 */
    s3?: (number|null);

    /** ivec6 s4 */
    s4?: (number|null);

    /** ivec6 s5 */
    s5?: (number|null);
}

/** Represents an ivec6. */
export class ivec6 implements Iivec6 {

    /**
     * Constructs a new ivec6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec6);

    /** ivec6 s0. */
    public s0: number;

    /** ivec6 s1. */
    public s1: number;

    /** ivec6 s2. */
    public s2: number;

    /** ivec6 s3. */
    public s3: number;

    /** ivec6 s4. */
    public s4: number;

    /** ivec6 s5. */
    public s5: number;

    /**
     * Encodes the specified ivec6 message. Does not implicitly {@link ivec6.verify|verify} messages.
     * @param message ivec6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec6;
}

/** Properties of an uvec6. */
export interface Iuvec6 {

    /** uvec6 s0 */
    s0?: (number|null);

    /** uvec6 s1 */
    s1?: (number|null);

    /** uvec6 s2 */
    s2?: (number|null);

    /** uvec6 s3 */
    s3?: (number|null);

    /** uvec6 s4 */
    s4?: (number|null);

    /** uvec6 s5 */
    s5?: (number|null);
}

/** Represents an uvec6. */
export class uvec6 implements Iuvec6 {

    /**
     * Constructs a new uvec6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec6);

    /** uvec6 s0. */
    public s0: number;

    /** uvec6 s1. */
    public s1: number;

    /** uvec6 s2. */
    public s2: number;

    /** uvec6 s3. */
    public s3: number;

    /** uvec6 s4. */
    public s4: number;

    /** uvec6 s5. */
    public s5: number;

    /**
     * Encodes the specified uvec6 message. Does not implicitly {@link uvec6.verify|verify} messages.
     * @param message uvec6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec6;
}

/** Properties of a vec7. */
export interface Ivec7 {

    /** vec7 s0 */
    s0?: (number|null);

    /** vec7 s1 */
    s1?: (number|null);

    /** vec7 s2 */
    s2?: (number|null);

    /** vec7 s3 */
    s3?: (number|null);

    /** vec7 s4 */
    s4?: (number|null);

    /** vec7 s5 */
    s5?: (number|null);

    /** vec7 s6 */
    s6?: (number|null);
}

/** Represents a vec7. */
export class vec7 implements Ivec7 {

    /**
     * Constructs a new vec7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec7);

    /** vec7 s0. */
    public s0: number;

    /** vec7 s1. */
    public s1: number;

    /** vec7 s2. */
    public s2: number;

    /** vec7 s3. */
    public s3: number;

    /** vec7 s4. */
    public s4: number;

    /** vec7 s5. */
    public s5: number;

    /** vec7 s6. */
    public s6: number;

    /**
     * Encodes the specified vec7 message. Does not implicitly {@link vec7.verify|verify} messages.
     * @param message vec7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec7;
}

/** Properties of a fvec7. */
export interface Ifvec7 {

    /** fvec7 s0 */
    s0?: (number|null);

    /** fvec7 s1 */
    s1?: (number|null);

    /** fvec7 s2 */
    s2?: (number|null);

    /** fvec7 s3 */
    s3?: (number|null);

    /** fvec7 s4 */
    s4?: (number|null);

    /** fvec7 s5 */
    s5?: (number|null);

    /** fvec7 s6 */
    s6?: (number|null);
}

/** Represents a fvec7. */
export class fvec7 implements Ifvec7 {

    /**
     * Constructs a new fvec7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec7);

    /** fvec7 s0. */
    public s0: number;

    /** fvec7 s1. */
    public s1: number;

    /** fvec7 s2. */
    public s2: number;

    /** fvec7 s3. */
    public s3: number;

    /** fvec7 s4. */
    public s4: number;

    /** fvec7 s5. */
    public s5: number;

    /** fvec7 s6. */
    public s6: number;

    /**
     * Encodes the specified fvec7 message. Does not implicitly {@link fvec7.verify|verify} messages.
     * @param message fvec7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec7;
}

/** Properties of an ivec7. */
export interface Iivec7 {

    /** ivec7 s0 */
    s0?: (number|null);

    /** ivec7 s1 */
    s1?: (number|null);

    /** ivec7 s2 */
    s2?: (number|null);

    /** ivec7 s3 */
    s3?: (number|null);

    /** ivec7 s4 */
    s4?: (number|null);

    /** ivec7 s5 */
    s5?: (number|null);

    /** ivec7 s6 */
    s6?: (number|null);
}

/** Represents an ivec7. */
export class ivec7 implements Iivec7 {

    /**
     * Constructs a new ivec7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec7);

    /** ivec7 s0. */
    public s0: number;

    /** ivec7 s1. */
    public s1: number;

    /** ivec7 s2. */
    public s2: number;

    /** ivec7 s3. */
    public s3: number;

    /** ivec7 s4. */
    public s4: number;

    /** ivec7 s5. */
    public s5: number;

    /** ivec7 s6. */
    public s6: number;

    /**
     * Encodes the specified ivec7 message. Does not implicitly {@link ivec7.verify|verify} messages.
     * @param message ivec7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec7;
}

/** Properties of an uvec7. */
export interface Iuvec7 {

    /** uvec7 s0 */
    s0?: (number|null);

    /** uvec7 s1 */
    s1?: (number|null);

    /** uvec7 s2 */
    s2?: (number|null);

    /** uvec7 s3 */
    s3?: (number|null);

    /** uvec7 s4 */
    s4?: (number|null);

    /** uvec7 s5 */
    s5?: (number|null);

    /** uvec7 s6 */
    s6?: (number|null);
}

/** Represents an uvec7. */
export class uvec7 implements Iuvec7 {

    /**
     * Constructs a new uvec7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec7);

    /** uvec7 s0. */
    public s0: number;

    /** uvec7 s1. */
    public s1: number;

    /** uvec7 s2. */
    public s2: number;

    /** uvec7 s3. */
    public s3: number;

    /** uvec7 s4. */
    public s4: number;

    /** uvec7 s5. */
    public s5: number;

    /** uvec7 s6. */
    public s6: number;

    /**
     * Encodes the specified uvec7 message. Does not implicitly {@link uvec7.verify|verify} messages.
     * @param message uvec7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec7;
}

/** Properties of a vec8. */
export interface Ivec8 {

    /** vec8 s0 */
    s0?: (number|null);

    /** vec8 s1 */
    s1?: (number|null);

    /** vec8 s2 */
    s2?: (number|null);

    /** vec8 s3 */
    s3?: (number|null);

    /** vec8 s4 */
    s4?: (number|null);

    /** vec8 s5 */
    s5?: (number|null);

    /** vec8 s6 */
    s6?: (number|null);

    /** vec8 s7 */
    s7?: (number|null);
}

/** Represents a vec8. */
export class vec8 implements Ivec8 {

    /**
     * Constructs a new vec8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec8);

    /** vec8 s0. */
    public s0: number;

    /** vec8 s1. */
    public s1: number;

    /** vec8 s2. */
    public s2: number;

    /** vec8 s3. */
    public s3: number;

    /** vec8 s4. */
    public s4: number;

    /** vec8 s5. */
    public s5: number;

    /** vec8 s6. */
    public s6: number;

    /** vec8 s7. */
    public s7: number;

    /**
     * Encodes the specified vec8 message. Does not implicitly {@link vec8.verify|verify} messages.
     * @param message vec8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec8;
}

/** Properties of a fvec8. */
export interface Ifvec8 {

    /** fvec8 s0 */
    s0?: (number|null);

    /** fvec8 s1 */
    s1?: (number|null);

    /** fvec8 s2 */
    s2?: (number|null);

    /** fvec8 s3 */
    s3?: (number|null);

    /** fvec8 s4 */
    s4?: (number|null);

    /** fvec8 s5 */
    s5?: (number|null);

    /** fvec8 s6 */
    s6?: (number|null);

    /** fvec8 s7 */
    s7?: (number|null);
}

/** Represents a fvec8. */
export class fvec8 implements Ifvec8 {

    /**
     * Constructs a new fvec8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec8);

    /** fvec8 s0. */
    public s0: number;

    /** fvec8 s1. */
    public s1: number;

    /** fvec8 s2. */
    public s2: number;

    /** fvec8 s3. */
    public s3: number;

    /** fvec8 s4. */
    public s4: number;

    /** fvec8 s5. */
    public s5: number;

    /** fvec8 s6. */
    public s6: number;

    /** fvec8 s7. */
    public s7: number;

    /**
     * Encodes the specified fvec8 message. Does not implicitly {@link fvec8.verify|verify} messages.
     * @param message fvec8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec8;
}

/** Properties of an ivec8. */
export interface Iivec8 {

    /** ivec8 s0 */
    s0?: (number|null);

    /** ivec8 s1 */
    s1?: (number|null);

    /** ivec8 s2 */
    s2?: (number|null);

    /** ivec8 s3 */
    s3?: (number|null);

    /** ivec8 s4 */
    s4?: (number|null);

    /** ivec8 s5 */
    s5?: (number|null);

    /** ivec8 s6 */
    s6?: (number|null);

    /** ivec8 s7 */
    s7?: (number|null);
}

/** Represents an ivec8. */
export class ivec8 implements Iivec8 {

    /**
     * Constructs a new ivec8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec8);

    /** ivec8 s0. */
    public s0: number;

    /** ivec8 s1. */
    public s1: number;

    /** ivec8 s2. */
    public s2: number;

    /** ivec8 s3. */
    public s3: number;

    /** ivec8 s4. */
    public s4: number;

    /** ivec8 s5. */
    public s5: number;

    /** ivec8 s6. */
    public s6: number;

    /** ivec8 s7. */
    public s7: number;

    /**
     * Encodes the specified ivec8 message. Does not implicitly {@link ivec8.verify|verify} messages.
     * @param message ivec8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec8;
}

/** Properties of an uvec8. */
export interface Iuvec8 {

    /** uvec8 s0 */
    s0?: (number|null);

    /** uvec8 s1 */
    s1?: (number|null);

    /** uvec8 s2 */
    s2?: (number|null);

    /** uvec8 s3 */
    s3?: (number|null);

    /** uvec8 s4 */
    s4?: (number|null);

    /** uvec8 s5 */
    s5?: (number|null);

    /** uvec8 s6 */
    s6?: (number|null);

    /** uvec8 s7 */
    s7?: (number|null);
}

/** Represents an uvec8. */
export class uvec8 implements Iuvec8 {

    /**
     * Constructs a new uvec8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec8);

    /** uvec8 s0. */
    public s0: number;

    /** uvec8 s1. */
    public s1: number;

    /** uvec8 s2. */
    public s2: number;

    /** uvec8 s3. */
    public s3: number;

    /** uvec8 s4. */
    public s4: number;

    /** uvec8 s5. */
    public s5: number;

    /** uvec8 s6. */
    public s6: number;

    /** uvec8 s7. */
    public s7: number;

    /**
     * Encodes the specified uvec8 message. Does not implicitly {@link uvec8.verify|verify} messages.
     * @param message uvec8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec8;
}

/** Properties of a vec9. */
export interface Ivec9 {

    /** vec9 s0 */
    s0?: (number|null);

    /** vec9 s1 */
    s1?: (number|null);

    /** vec9 s2 */
    s2?: (number|null);

    /** vec9 s3 */
    s3?: (number|null);

    /** vec9 s4 */
    s4?: (number|null);

    /** vec9 s5 */
    s5?: (number|null);

    /** vec9 s6 */
    s6?: (number|null);

    /** vec9 s7 */
    s7?: (number|null);

    /** vec9 s8 */
    s8?: (number|null);
}

/** Represents a vec9. */
export class vec9 implements Ivec9 {

    /**
     * Constructs a new vec9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec9);

    /** vec9 s0. */
    public s0: number;

    /** vec9 s1. */
    public s1: number;

    /** vec9 s2. */
    public s2: number;

    /** vec9 s3. */
    public s3: number;

    /** vec9 s4. */
    public s4: number;

    /** vec9 s5. */
    public s5: number;

    /** vec9 s6. */
    public s6: number;

    /** vec9 s7. */
    public s7: number;

    /** vec9 s8. */
    public s8: number;

    /**
     * Encodes the specified vec9 message. Does not implicitly {@link vec9.verify|verify} messages.
     * @param message vec9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec9;
}

/** Properties of a fvec9. */
export interface Ifvec9 {

    /** fvec9 s0 */
    s0?: (number|null);

    /** fvec9 s1 */
    s1?: (number|null);

    /** fvec9 s2 */
    s2?: (number|null);

    /** fvec9 s3 */
    s3?: (number|null);

    /** fvec9 s4 */
    s4?: (number|null);

    /** fvec9 s5 */
    s5?: (number|null);

    /** fvec9 s6 */
    s6?: (number|null);

    /** fvec9 s7 */
    s7?: (number|null);

    /** fvec9 s8 */
    s8?: (number|null);
}

/** Represents a fvec9. */
export class fvec9 implements Ifvec9 {

    /**
     * Constructs a new fvec9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec9);

    /** fvec9 s0. */
    public s0: number;

    /** fvec9 s1. */
    public s1: number;

    /** fvec9 s2. */
    public s2: number;

    /** fvec9 s3. */
    public s3: number;

    /** fvec9 s4. */
    public s4: number;

    /** fvec9 s5. */
    public s5: number;

    /** fvec9 s6. */
    public s6: number;

    /** fvec9 s7. */
    public s7: number;

    /** fvec9 s8. */
    public s8: number;

    /**
     * Encodes the specified fvec9 message. Does not implicitly {@link fvec9.verify|verify} messages.
     * @param message fvec9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec9;
}

/** Properties of an ivec9. */
export interface Iivec9 {

    /** ivec9 s0 */
    s0?: (number|null);

    /** ivec9 s1 */
    s1?: (number|null);

    /** ivec9 s2 */
    s2?: (number|null);

    /** ivec9 s3 */
    s3?: (number|null);

    /** ivec9 s4 */
    s4?: (number|null);

    /** ivec9 s5 */
    s5?: (number|null);

    /** ivec9 s6 */
    s6?: (number|null);

    /** ivec9 s7 */
    s7?: (number|null);

    /** ivec9 s8 */
    s8?: (number|null);
}

/** Represents an ivec9. */
export class ivec9 implements Iivec9 {

    /**
     * Constructs a new ivec9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec9);

    /** ivec9 s0. */
    public s0: number;

    /** ivec9 s1. */
    public s1: number;

    /** ivec9 s2. */
    public s2: number;

    /** ivec9 s3. */
    public s3: number;

    /** ivec9 s4. */
    public s4: number;

    /** ivec9 s5. */
    public s5: number;

    /** ivec9 s6. */
    public s6: number;

    /** ivec9 s7. */
    public s7: number;

    /** ivec9 s8. */
    public s8: number;

    /**
     * Encodes the specified ivec9 message. Does not implicitly {@link ivec9.verify|verify} messages.
     * @param message ivec9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec9;
}

/** Properties of an uvec9. */
export interface Iuvec9 {

    /** uvec9 s0 */
    s0?: (number|null);

    /** uvec9 s1 */
    s1?: (number|null);

    /** uvec9 s2 */
    s2?: (number|null);

    /** uvec9 s3 */
    s3?: (number|null);

    /** uvec9 s4 */
    s4?: (number|null);

    /** uvec9 s5 */
    s5?: (number|null);

    /** uvec9 s6 */
    s6?: (number|null);

    /** uvec9 s7 */
    s7?: (number|null);

    /** uvec9 s8 */
    s8?: (number|null);
}

/** Represents an uvec9. */
export class uvec9 implements Iuvec9 {

    /**
     * Constructs a new uvec9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec9);

    /** uvec9 s0. */
    public s0: number;

    /** uvec9 s1. */
    public s1: number;

    /** uvec9 s2. */
    public s2: number;

    /** uvec9 s3. */
    public s3: number;

    /** uvec9 s4. */
    public s4: number;

    /** uvec9 s5. */
    public s5: number;

    /** uvec9 s6. */
    public s6: number;

    /** uvec9 s7. */
    public s7: number;

    /** uvec9 s8. */
    public s8: number;

    /**
     * Encodes the specified uvec9 message. Does not implicitly {@link uvec9.verify|verify} messages.
     * @param message uvec9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec9;
}

/** Properties of a vec10. */
export interface Ivec10 {

    /** vec10 s0 */
    s0?: (number|null);

    /** vec10 s1 */
    s1?: (number|null);

    /** vec10 s2 */
    s2?: (number|null);

    /** vec10 s3 */
    s3?: (number|null);

    /** vec10 s4 */
    s4?: (number|null);

    /** vec10 s5 */
    s5?: (number|null);

    /** vec10 s6 */
    s6?: (number|null);

    /** vec10 s7 */
    s7?: (number|null);

    /** vec10 s8 */
    s8?: (number|null);

    /** vec10 s9 */
    s9?: (number|null);
}

/** Represents a vec10. */
export class vec10 implements Ivec10 {

    /**
     * Constructs a new vec10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec10);

    /** vec10 s0. */
    public s0: number;

    /** vec10 s1. */
    public s1: number;

    /** vec10 s2. */
    public s2: number;

    /** vec10 s3. */
    public s3: number;

    /** vec10 s4. */
    public s4: number;

    /** vec10 s5. */
    public s5: number;

    /** vec10 s6. */
    public s6: number;

    /** vec10 s7. */
    public s7: number;

    /** vec10 s8. */
    public s8: number;

    /** vec10 s9. */
    public s9: number;

    /**
     * Encodes the specified vec10 message. Does not implicitly {@link vec10.verify|verify} messages.
     * @param message vec10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec10;
}

/** Properties of a fvec10. */
export interface Ifvec10 {

    /** fvec10 s0 */
    s0?: (number|null);

    /** fvec10 s1 */
    s1?: (number|null);

    /** fvec10 s2 */
    s2?: (number|null);

    /** fvec10 s3 */
    s3?: (number|null);

    /** fvec10 s4 */
    s4?: (number|null);

    /** fvec10 s5 */
    s5?: (number|null);

    /** fvec10 s6 */
    s6?: (number|null);

    /** fvec10 s7 */
    s7?: (number|null);

    /** fvec10 s8 */
    s8?: (number|null);

    /** fvec10 s9 */
    s9?: (number|null);
}

/** Represents a fvec10. */
export class fvec10 implements Ifvec10 {

    /**
     * Constructs a new fvec10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec10);

    /** fvec10 s0. */
    public s0: number;

    /** fvec10 s1. */
    public s1: number;

    /** fvec10 s2. */
    public s2: number;

    /** fvec10 s3. */
    public s3: number;

    /** fvec10 s4. */
    public s4: number;

    /** fvec10 s5. */
    public s5: number;

    /** fvec10 s6. */
    public s6: number;

    /** fvec10 s7. */
    public s7: number;

    /** fvec10 s8. */
    public s8: number;

    /** fvec10 s9. */
    public s9: number;

    /**
     * Encodes the specified fvec10 message. Does not implicitly {@link fvec10.verify|verify} messages.
     * @param message fvec10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec10;
}

/** Properties of an ivec10. */
export interface Iivec10 {

    /** ivec10 s0 */
    s0?: (number|null);

    /** ivec10 s1 */
    s1?: (number|null);

    /** ivec10 s2 */
    s2?: (number|null);

    /** ivec10 s3 */
    s3?: (number|null);

    /** ivec10 s4 */
    s4?: (number|null);

    /** ivec10 s5 */
    s5?: (number|null);

    /** ivec10 s6 */
    s6?: (number|null);

    /** ivec10 s7 */
    s7?: (number|null);

    /** ivec10 s8 */
    s8?: (number|null);

    /** ivec10 s9 */
    s9?: (number|null);
}

/** Represents an ivec10. */
export class ivec10 implements Iivec10 {

    /**
     * Constructs a new ivec10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec10);

    /** ivec10 s0. */
    public s0: number;

    /** ivec10 s1. */
    public s1: number;

    /** ivec10 s2. */
    public s2: number;

    /** ivec10 s3. */
    public s3: number;

    /** ivec10 s4. */
    public s4: number;

    /** ivec10 s5. */
    public s5: number;

    /** ivec10 s6. */
    public s6: number;

    /** ivec10 s7. */
    public s7: number;

    /** ivec10 s8. */
    public s8: number;

    /** ivec10 s9. */
    public s9: number;

    /**
     * Encodes the specified ivec10 message. Does not implicitly {@link ivec10.verify|verify} messages.
     * @param message ivec10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec10;
}

/** Properties of an uvec10. */
export interface Iuvec10 {

    /** uvec10 s0 */
    s0?: (number|null);

    /** uvec10 s1 */
    s1?: (number|null);

    /** uvec10 s2 */
    s2?: (number|null);

    /** uvec10 s3 */
    s3?: (number|null);

    /** uvec10 s4 */
    s4?: (number|null);

    /** uvec10 s5 */
    s5?: (number|null);

    /** uvec10 s6 */
    s6?: (number|null);

    /** uvec10 s7 */
    s7?: (number|null);

    /** uvec10 s8 */
    s8?: (number|null);

    /** uvec10 s9 */
    s9?: (number|null);
}

/** Represents an uvec10. */
export class uvec10 implements Iuvec10 {

    /**
     * Constructs a new uvec10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec10);

    /** uvec10 s0. */
    public s0: number;

    /** uvec10 s1. */
    public s1: number;

    /** uvec10 s2. */
    public s2: number;

    /** uvec10 s3. */
    public s3: number;

    /** uvec10 s4. */
    public s4: number;

    /** uvec10 s5. */
    public s5: number;

    /** uvec10 s6. */
    public s6: number;

    /** uvec10 s7. */
    public s7: number;

    /** uvec10 s8. */
    public s8: number;

    /** uvec10 s9. */
    public s9: number;

    /**
     * Encodes the specified uvec10 message. Does not implicitly {@link uvec10.verify|verify} messages.
     * @param message uvec10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec10;
}

/** Properties of a vec11. */
export interface Ivec11 {

    /** vec11 s0 */
    s0?: (number|null);

    /** vec11 s1 */
    s1?: (number|null);

    /** vec11 s2 */
    s2?: (number|null);

    /** vec11 s3 */
    s3?: (number|null);

    /** vec11 s4 */
    s4?: (number|null);

    /** vec11 s5 */
    s5?: (number|null);

    /** vec11 s6 */
    s6?: (number|null);

    /** vec11 s7 */
    s7?: (number|null);

    /** vec11 s8 */
    s8?: (number|null);

    /** vec11 s9 */
    s9?: (number|null);

    /** vec11 sa */
    sa?: (number|null);
}

/** Represents a vec11. */
export class vec11 implements Ivec11 {

    /**
     * Constructs a new vec11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec11);

    /** vec11 s0. */
    public s0: number;

    /** vec11 s1. */
    public s1: number;

    /** vec11 s2. */
    public s2: number;

    /** vec11 s3. */
    public s3: number;

    /** vec11 s4. */
    public s4: number;

    /** vec11 s5. */
    public s5: number;

    /** vec11 s6. */
    public s6: number;

    /** vec11 s7. */
    public s7: number;

    /** vec11 s8. */
    public s8: number;

    /** vec11 s9. */
    public s9: number;

    /** vec11 sa. */
    public sa: number;

    /**
     * Encodes the specified vec11 message. Does not implicitly {@link vec11.verify|verify} messages.
     * @param message vec11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec11;
}

/** Properties of a fvec11. */
export interface Ifvec11 {

    /** fvec11 s0 */
    s0?: (number|null);

    /** fvec11 s1 */
    s1?: (number|null);

    /** fvec11 s2 */
    s2?: (number|null);

    /** fvec11 s3 */
    s3?: (number|null);

    /** fvec11 s4 */
    s4?: (number|null);

    /** fvec11 s5 */
    s5?: (number|null);

    /** fvec11 s6 */
    s6?: (number|null);

    /** fvec11 s7 */
    s7?: (number|null);

    /** fvec11 s8 */
    s8?: (number|null);

    /** fvec11 s9 */
    s9?: (number|null);

    /** fvec11 sa */
    sa?: (number|null);
}

/** Represents a fvec11. */
export class fvec11 implements Ifvec11 {

    /**
     * Constructs a new fvec11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec11);

    /** fvec11 s0. */
    public s0: number;

    /** fvec11 s1. */
    public s1: number;

    /** fvec11 s2. */
    public s2: number;

    /** fvec11 s3. */
    public s3: number;

    /** fvec11 s4. */
    public s4: number;

    /** fvec11 s5. */
    public s5: number;

    /** fvec11 s6. */
    public s6: number;

    /** fvec11 s7. */
    public s7: number;

    /** fvec11 s8. */
    public s8: number;

    /** fvec11 s9. */
    public s9: number;

    /** fvec11 sa. */
    public sa: number;

    /**
     * Encodes the specified fvec11 message. Does not implicitly {@link fvec11.verify|verify} messages.
     * @param message fvec11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec11;
}

/** Properties of an ivec11. */
export interface Iivec11 {

    /** ivec11 s0 */
    s0?: (number|null);

    /** ivec11 s1 */
    s1?: (number|null);

    /** ivec11 s2 */
    s2?: (number|null);

    /** ivec11 s3 */
    s3?: (number|null);

    /** ivec11 s4 */
    s4?: (number|null);

    /** ivec11 s5 */
    s5?: (number|null);

    /** ivec11 s6 */
    s6?: (number|null);

    /** ivec11 s7 */
    s7?: (number|null);

    /** ivec11 s8 */
    s8?: (number|null);

    /** ivec11 s9 */
    s9?: (number|null);

    /** ivec11 sa */
    sa?: (number|null);
}

/** Represents an ivec11. */
export class ivec11 implements Iivec11 {

    /**
     * Constructs a new ivec11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec11);

    /** ivec11 s0. */
    public s0: number;

    /** ivec11 s1. */
    public s1: number;

    /** ivec11 s2. */
    public s2: number;

    /** ivec11 s3. */
    public s3: number;

    /** ivec11 s4. */
    public s4: number;

    /** ivec11 s5. */
    public s5: number;

    /** ivec11 s6. */
    public s6: number;

    /** ivec11 s7. */
    public s7: number;

    /** ivec11 s8. */
    public s8: number;

    /** ivec11 s9. */
    public s9: number;

    /** ivec11 sa. */
    public sa: number;

    /**
     * Encodes the specified ivec11 message. Does not implicitly {@link ivec11.verify|verify} messages.
     * @param message ivec11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec11;
}

/** Properties of an uvec11. */
export interface Iuvec11 {

    /** uvec11 s0 */
    s0?: (number|null);

    /** uvec11 s1 */
    s1?: (number|null);

    /** uvec11 s2 */
    s2?: (number|null);

    /** uvec11 s3 */
    s3?: (number|null);

    /** uvec11 s4 */
    s4?: (number|null);

    /** uvec11 s5 */
    s5?: (number|null);

    /** uvec11 s6 */
    s6?: (number|null);

    /** uvec11 s7 */
    s7?: (number|null);

    /** uvec11 s8 */
    s8?: (number|null);

    /** uvec11 s9 */
    s9?: (number|null);

    /** uvec11 sa */
    sa?: (number|null);
}

/** Represents an uvec11. */
export class uvec11 implements Iuvec11 {

    /**
     * Constructs a new uvec11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec11);

    /** uvec11 s0. */
    public s0: number;

    /** uvec11 s1. */
    public s1: number;

    /** uvec11 s2. */
    public s2: number;

    /** uvec11 s3. */
    public s3: number;

    /** uvec11 s4. */
    public s4: number;

    /** uvec11 s5. */
    public s5: number;

    /** uvec11 s6. */
    public s6: number;

    /** uvec11 s7. */
    public s7: number;

    /** uvec11 s8. */
    public s8: number;

    /** uvec11 s9. */
    public s9: number;

    /** uvec11 sa. */
    public sa: number;

    /**
     * Encodes the specified uvec11 message. Does not implicitly {@link uvec11.verify|verify} messages.
     * @param message uvec11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec11;
}

/** Properties of a vec12. */
export interface Ivec12 {

    /** vec12 s0 */
    s0?: (number|null);

    /** vec12 s1 */
    s1?: (number|null);

    /** vec12 s2 */
    s2?: (number|null);

    /** vec12 s3 */
    s3?: (number|null);

    /** vec12 s4 */
    s4?: (number|null);

    /** vec12 s5 */
    s5?: (number|null);

    /** vec12 s6 */
    s6?: (number|null);

    /** vec12 s7 */
    s7?: (number|null);

    /** vec12 s8 */
    s8?: (number|null);

    /** vec12 s9 */
    s9?: (number|null);

    /** vec12 sa */
    sa?: (number|null);

    /** vec12 sb */
    sb?: (number|null);
}

/** Represents a vec12. */
export class vec12 implements Ivec12 {

    /**
     * Constructs a new vec12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec12);

    /** vec12 s0. */
    public s0: number;

    /** vec12 s1. */
    public s1: number;

    /** vec12 s2. */
    public s2: number;

    /** vec12 s3. */
    public s3: number;

    /** vec12 s4. */
    public s4: number;

    /** vec12 s5. */
    public s5: number;

    /** vec12 s6. */
    public s6: number;

    /** vec12 s7. */
    public s7: number;

    /** vec12 s8. */
    public s8: number;

    /** vec12 s9. */
    public s9: number;

    /** vec12 sa. */
    public sa: number;

    /** vec12 sb. */
    public sb: number;

    /**
     * Encodes the specified vec12 message. Does not implicitly {@link vec12.verify|verify} messages.
     * @param message vec12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec12;
}

/** Properties of a fvec12. */
export interface Ifvec12 {

    /** fvec12 s0 */
    s0?: (number|null);

    /** fvec12 s1 */
    s1?: (number|null);

    /** fvec12 s2 */
    s2?: (number|null);

    /** fvec12 s3 */
    s3?: (number|null);

    /** fvec12 s4 */
    s4?: (number|null);

    /** fvec12 s5 */
    s5?: (number|null);

    /** fvec12 s6 */
    s6?: (number|null);

    /** fvec12 s7 */
    s7?: (number|null);

    /** fvec12 s8 */
    s8?: (number|null);

    /** fvec12 s9 */
    s9?: (number|null);

    /** fvec12 sa */
    sa?: (number|null);

    /** fvec12 sb */
    sb?: (number|null);
}

/** Represents a fvec12. */
export class fvec12 implements Ifvec12 {

    /**
     * Constructs a new fvec12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec12);

    /** fvec12 s0. */
    public s0: number;

    /** fvec12 s1. */
    public s1: number;

    /** fvec12 s2. */
    public s2: number;

    /** fvec12 s3. */
    public s3: number;

    /** fvec12 s4. */
    public s4: number;

    /** fvec12 s5. */
    public s5: number;

    /** fvec12 s6. */
    public s6: number;

    /** fvec12 s7. */
    public s7: number;

    /** fvec12 s8. */
    public s8: number;

    /** fvec12 s9. */
    public s9: number;

    /** fvec12 sa. */
    public sa: number;

    /** fvec12 sb. */
    public sb: number;

    /**
     * Encodes the specified fvec12 message. Does not implicitly {@link fvec12.verify|verify} messages.
     * @param message fvec12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec12;
}

/** Properties of an ivec12. */
export interface Iivec12 {

    /** ivec12 s0 */
    s0?: (number|null);

    /** ivec12 s1 */
    s1?: (number|null);

    /** ivec12 s2 */
    s2?: (number|null);

    /** ivec12 s3 */
    s3?: (number|null);

    /** ivec12 s4 */
    s4?: (number|null);

    /** ivec12 s5 */
    s5?: (number|null);

    /** ivec12 s6 */
    s6?: (number|null);

    /** ivec12 s7 */
    s7?: (number|null);

    /** ivec12 s8 */
    s8?: (number|null);

    /** ivec12 s9 */
    s9?: (number|null);

    /** ivec12 sa */
    sa?: (number|null);

    /** ivec12 sb */
    sb?: (number|null);
}

/** Represents an ivec12. */
export class ivec12 implements Iivec12 {

    /**
     * Constructs a new ivec12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec12);

    /** ivec12 s0. */
    public s0: number;

    /** ivec12 s1. */
    public s1: number;

    /** ivec12 s2. */
    public s2: number;

    /** ivec12 s3. */
    public s3: number;

    /** ivec12 s4. */
    public s4: number;

    /** ivec12 s5. */
    public s5: number;

    /** ivec12 s6. */
    public s6: number;

    /** ivec12 s7. */
    public s7: number;

    /** ivec12 s8. */
    public s8: number;

    /** ivec12 s9. */
    public s9: number;

    /** ivec12 sa. */
    public sa: number;

    /** ivec12 sb. */
    public sb: number;

    /**
     * Encodes the specified ivec12 message. Does not implicitly {@link ivec12.verify|verify} messages.
     * @param message ivec12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec12;
}

/** Properties of an uvec12. */
export interface Iuvec12 {

    /** uvec12 s0 */
    s0?: (number|null);

    /** uvec12 s1 */
    s1?: (number|null);

    /** uvec12 s2 */
    s2?: (number|null);

    /** uvec12 s3 */
    s3?: (number|null);

    /** uvec12 s4 */
    s4?: (number|null);

    /** uvec12 s5 */
    s5?: (number|null);

    /** uvec12 s6 */
    s6?: (number|null);

    /** uvec12 s7 */
    s7?: (number|null);

    /** uvec12 s8 */
    s8?: (number|null);

    /** uvec12 s9 */
    s9?: (number|null);

    /** uvec12 sa */
    sa?: (number|null);

    /** uvec12 sb */
    sb?: (number|null);
}

/** Represents an uvec12. */
export class uvec12 implements Iuvec12 {

    /**
     * Constructs a new uvec12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec12);

    /** uvec12 s0. */
    public s0: number;

    /** uvec12 s1. */
    public s1: number;

    /** uvec12 s2. */
    public s2: number;

    /** uvec12 s3. */
    public s3: number;

    /** uvec12 s4. */
    public s4: number;

    /** uvec12 s5. */
    public s5: number;

    /** uvec12 s6. */
    public s6: number;

    /** uvec12 s7. */
    public s7: number;

    /** uvec12 s8. */
    public s8: number;

    /** uvec12 s9. */
    public s9: number;

    /** uvec12 sa. */
    public sa: number;

    /** uvec12 sb. */
    public sb: number;

    /**
     * Encodes the specified uvec12 message. Does not implicitly {@link uvec12.verify|verify} messages.
     * @param message uvec12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec12;
}

/** Properties of a vec13. */
export interface Ivec13 {

    /** vec13 s0 */
    s0?: (number|null);

    /** vec13 s1 */
    s1?: (number|null);

    /** vec13 s2 */
    s2?: (number|null);

    /** vec13 s3 */
    s3?: (number|null);

    /** vec13 s4 */
    s4?: (number|null);

    /** vec13 s5 */
    s5?: (number|null);

    /** vec13 s6 */
    s6?: (number|null);

    /** vec13 s7 */
    s7?: (number|null);

    /** vec13 s8 */
    s8?: (number|null);

    /** vec13 s9 */
    s9?: (number|null);

    /** vec13 sa */
    sa?: (number|null);

    /** vec13 sb */
    sb?: (number|null);

    /** vec13 sc */
    sc?: (number|null);
}

/** Represents a vec13. */
export class vec13 implements Ivec13 {

    /**
     * Constructs a new vec13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec13);

    /** vec13 s0. */
    public s0: number;

    /** vec13 s1. */
    public s1: number;

    /** vec13 s2. */
    public s2: number;

    /** vec13 s3. */
    public s3: number;

    /** vec13 s4. */
    public s4: number;

    /** vec13 s5. */
    public s5: number;

    /** vec13 s6. */
    public s6: number;

    /** vec13 s7. */
    public s7: number;

    /** vec13 s8. */
    public s8: number;

    /** vec13 s9. */
    public s9: number;

    /** vec13 sa. */
    public sa: number;

    /** vec13 sb. */
    public sb: number;

    /** vec13 sc. */
    public sc: number;

    /**
     * Encodes the specified vec13 message. Does not implicitly {@link vec13.verify|verify} messages.
     * @param message vec13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec13;
}

/** Properties of a fvec13. */
export interface Ifvec13 {

    /** fvec13 s0 */
    s0?: (number|null);

    /** fvec13 s1 */
    s1?: (number|null);

    /** fvec13 s2 */
    s2?: (number|null);

    /** fvec13 s3 */
    s3?: (number|null);

    /** fvec13 s4 */
    s4?: (number|null);

    /** fvec13 s5 */
    s5?: (number|null);

    /** fvec13 s6 */
    s6?: (number|null);

    /** fvec13 s7 */
    s7?: (number|null);

    /** fvec13 s8 */
    s8?: (number|null);

    /** fvec13 s9 */
    s9?: (number|null);

    /** fvec13 sa */
    sa?: (number|null);

    /** fvec13 sb */
    sb?: (number|null);

    /** fvec13 sc */
    sc?: (number|null);
}

/** Represents a fvec13. */
export class fvec13 implements Ifvec13 {

    /**
     * Constructs a new fvec13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec13);

    /** fvec13 s0. */
    public s0: number;

    /** fvec13 s1. */
    public s1: number;

    /** fvec13 s2. */
    public s2: number;

    /** fvec13 s3. */
    public s3: number;

    /** fvec13 s4. */
    public s4: number;

    /** fvec13 s5. */
    public s5: number;

    /** fvec13 s6. */
    public s6: number;

    /** fvec13 s7. */
    public s7: number;

    /** fvec13 s8. */
    public s8: number;

    /** fvec13 s9. */
    public s9: number;

    /** fvec13 sa. */
    public sa: number;

    /** fvec13 sb. */
    public sb: number;

    /** fvec13 sc. */
    public sc: number;

    /**
     * Encodes the specified fvec13 message. Does not implicitly {@link fvec13.verify|verify} messages.
     * @param message fvec13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec13;
}

/** Properties of an ivec13. */
export interface Iivec13 {

    /** ivec13 s0 */
    s0?: (number|null);

    /** ivec13 s1 */
    s1?: (number|null);

    /** ivec13 s2 */
    s2?: (number|null);

    /** ivec13 s3 */
    s3?: (number|null);

    /** ivec13 s4 */
    s4?: (number|null);

    /** ivec13 s5 */
    s5?: (number|null);

    /** ivec13 s6 */
    s6?: (number|null);

    /** ivec13 s7 */
    s7?: (number|null);

    /** ivec13 s8 */
    s8?: (number|null);

    /** ivec13 s9 */
    s9?: (number|null);

    /** ivec13 sa */
    sa?: (number|null);

    /** ivec13 sb */
    sb?: (number|null);

    /** ivec13 sc */
    sc?: (number|null);
}

/** Represents an ivec13. */
export class ivec13 implements Iivec13 {

    /**
     * Constructs a new ivec13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec13);

    /** ivec13 s0. */
    public s0: number;

    /** ivec13 s1. */
    public s1: number;

    /** ivec13 s2. */
    public s2: number;

    /** ivec13 s3. */
    public s3: number;

    /** ivec13 s4. */
    public s4: number;

    /** ivec13 s5. */
    public s5: number;

    /** ivec13 s6. */
    public s6: number;

    /** ivec13 s7. */
    public s7: number;

    /** ivec13 s8. */
    public s8: number;

    /** ivec13 s9. */
    public s9: number;

    /** ivec13 sa. */
    public sa: number;

    /** ivec13 sb. */
    public sb: number;

    /** ivec13 sc. */
    public sc: number;

    /**
     * Encodes the specified ivec13 message. Does not implicitly {@link ivec13.verify|verify} messages.
     * @param message ivec13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec13;
}

/** Properties of an uvec13. */
export interface Iuvec13 {

    /** uvec13 s0 */
    s0?: (number|null);

    /** uvec13 s1 */
    s1?: (number|null);

    /** uvec13 s2 */
    s2?: (number|null);

    /** uvec13 s3 */
    s3?: (number|null);

    /** uvec13 s4 */
    s4?: (number|null);

    /** uvec13 s5 */
    s5?: (number|null);

    /** uvec13 s6 */
    s6?: (number|null);

    /** uvec13 s7 */
    s7?: (number|null);

    /** uvec13 s8 */
    s8?: (number|null);

    /** uvec13 s9 */
    s9?: (number|null);

    /** uvec13 sa */
    sa?: (number|null);

    /** uvec13 sb */
    sb?: (number|null);

    /** uvec13 sc */
    sc?: (number|null);
}

/** Represents an uvec13. */
export class uvec13 implements Iuvec13 {

    /**
     * Constructs a new uvec13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec13);

    /** uvec13 s0. */
    public s0: number;

    /** uvec13 s1. */
    public s1: number;

    /** uvec13 s2. */
    public s2: number;

    /** uvec13 s3. */
    public s3: number;

    /** uvec13 s4. */
    public s4: number;

    /** uvec13 s5. */
    public s5: number;

    /** uvec13 s6. */
    public s6: number;

    /** uvec13 s7. */
    public s7: number;

    /** uvec13 s8. */
    public s8: number;

    /** uvec13 s9. */
    public s9: number;

    /** uvec13 sa. */
    public sa: number;

    /** uvec13 sb. */
    public sb: number;

    /** uvec13 sc. */
    public sc: number;

    /**
     * Encodes the specified uvec13 message. Does not implicitly {@link uvec13.verify|verify} messages.
     * @param message uvec13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec13;
}

/** Properties of a vec14. */
export interface Ivec14 {

    /** vec14 s0 */
    s0?: (number|null);

    /** vec14 s1 */
    s1?: (number|null);

    /** vec14 s2 */
    s2?: (number|null);

    /** vec14 s3 */
    s3?: (number|null);

    /** vec14 s4 */
    s4?: (number|null);

    /** vec14 s5 */
    s5?: (number|null);

    /** vec14 s6 */
    s6?: (number|null);

    /** vec14 s7 */
    s7?: (number|null);

    /** vec14 s8 */
    s8?: (number|null);

    /** vec14 s9 */
    s9?: (number|null);

    /** vec14 sa */
    sa?: (number|null);

    /** vec14 sb */
    sb?: (number|null);

    /** vec14 sc */
    sc?: (number|null);

    /** vec14 sd */
    sd?: (number|null);
}

/** Represents a vec14. */
export class vec14 implements Ivec14 {

    /**
     * Constructs a new vec14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec14);

    /** vec14 s0. */
    public s0: number;

    /** vec14 s1. */
    public s1: number;

    /** vec14 s2. */
    public s2: number;

    /** vec14 s3. */
    public s3: number;

    /** vec14 s4. */
    public s4: number;

    /** vec14 s5. */
    public s5: number;

    /** vec14 s6. */
    public s6: number;

    /** vec14 s7. */
    public s7: number;

    /** vec14 s8. */
    public s8: number;

    /** vec14 s9. */
    public s9: number;

    /** vec14 sa. */
    public sa: number;

    /** vec14 sb. */
    public sb: number;

    /** vec14 sc. */
    public sc: number;

    /** vec14 sd. */
    public sd: number;

    /**
     * Encodes the specified vec14 message. Does not implicitly {@link vec14.verify|verify} messages.
     * @param message vec14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec14;
}

/** Properties of a fvec14. */
export interface Ifvec14 {

    /** fvec14 s0 */
    s0?: (number|null);

    /** fvec14 s1 */
    s1?: (number|null);

    /** fvec14 s2 */
    s2?: (number|null);

    /** fvec14 s3 */
    s3?: (number|null);

    /** fvec14 s4 */
    s4?: (number|null);

    /** fvec14 s5 */
    s5?: (number|null);

    /** fvec14 s6 */
    s6?: (number|null);

    /** fvec14 s7 */
    s7?: (number|null);

    /** fvec14 s8 */
    s8?: (number|null);

    /** fvec14 s9 */
    s9?: (number|null);

    /** fvec14 sa */
    sa?: (number|null);

    /** fvec14 sb */
    sb?: (number|null);

    /** fvec14 sc */
    sc?: (number|null);

    /** fvec14 sd */
    sd?: (number|null);
}

/** Represents a fvec14. */
export class fvec14 implements Ifvec14 {

    /**
     * Constructs a new fvec14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec14);

    /** fvec14 s0. */
    public s0: number;

    /** fvec14 s1. */
    public s1: number;

    /** fvec14 s2. */
    public s2: number;

    /** fvec14 s3. */
    public s3: number;

    /** fvec14 s4. */
    public s4: number;

    /** fvec14 s5. */
    public s5: number;

    /** fvec14 s6. */
    public s6: number;

    /** fvec14 s7. */
    public s7: number;

    /** fvec14 s8. */
    public s8: number;

    /** fvec14 s9. */
    public s9: number;

    /** fvec14 sa. */
    public sa: number;

    /** fvec14 sb. */
    public sb: number;

    /** fvec14 sc. */
    public sc: number;

    /** fvec14 sd. */
    public sd: number;

    /**
     * Encodes the specified fvec14 message. Does not implicitly {@link fvec14.verify|verify} messages.
     * @param message fvec14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec14;
}

/** Properties of an ivec14. */
export interface Iivec14 {

    /** ivec14 s0 */
    s0?: (number|null);

    /** ivec14 s1 */
    s1?: (number|null);

    /** ivec14 s2 */
    s2?: (number|null);

    /** ivec14 s3 */
    s3?: (number|null);

    /** ivec14 s4 */
    s4?: (number|null);

    /** ivec14 s5 */
    s5?: (number|null);

    /** ivec14 s6 */
    s6?: (number|null);

    /** ivec14 s7 */
    s7?: (number|null);

    /** ivec14 s8 */
    s8?: (number|null);

    /** ivec14 s9 */
    s9?: (number|null);

    /** ivec14 sa */
    sa?: (number|null);

    /** ivec14 sb */
    sb?: (number|null);

    /** ivec14 sc */
    sc?: (number|null);

    /** ivec14 sd */
    sd?: (number|null);
}

/** Represents an ivec14. */
export class ivec14 implements Iivec14 {

    /**
     * Constructs a new ivec14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec14);

    /** ivec14 s0. */
    public s0: number;

    /** ivec14 s1. */
    public s1: number;

    /** ivec14 s2. */
    public s2: number;

    /** ivec14 s3. */
    public s3: number;

    /** ivec14 s4. */
    public s4: number;

    /** ivec14 s5. */
    public s5: number;

    /** ivec14 s6. */
    public s6: number;

    /** ivec14 s7. */
    public s7: number;

    /** ivec14 s8. */
    public s8: number;

    /** ivec14 s9. */
    public s9: number;

    /** ivec14 sa. */
    public sa: number;

    /** ivec14 sb. */
    public sb: number;

    /** ivec14 sc. */
    public sc: number;

    /** ivec14 sd. */
    public sd: number;

    /**
     * Encodes the specified ivec14 message. Does not implicitly {@link ivec14.verify|verify} messages.
     * @param message ivec14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec14;
}

/** Properties of an uvec14. */
export interface Iuvec14 {

    /** uvec14 s0 */
    s0?: (number|null);

    /** uvec14 s1 */
    s1?: (number|null);

    /** uvec14 s2 */
    s2?: (number|null);

    /** uvec14 s3 */
    s3?: (number|null);

    /** uvec14 s4 */
    s4?: (number|null);

    /** uvec14 s5 */
    s5?: (number|null);

    /** uvec14 s6 */
    s6?: (number|null);

    /** uvec14 s7 */
    s7?: (number|null);

    /** uvec14 s8 */
    s8?: (number|null);

    /** uvec14 s9 */
    s9?: (number|null);

    /** uvec14 sa */
    sa?: (number|null);

    /** uvec14 sb */
    sb?: (number|null);

    /** uvec14 sc */
    sc?: (number|null);

    /** uvec14 sd */
    sd?: (number|null);
}

/** Represents an uvec14. */
export class uvec14 implements Iuvec14 {

    /**
     * Constructs a new uvec14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec14);

    /** uvec14 s0. */
    public s0: number;

    /** uvec14 s1. */
    public s1: number;

    /** uvec14 s2. */
    public s2: number;

    /** uvec14 s3. */
    public s3: number;

    /** uvec14 s4. */
    public s4: number;

    /** uvec14 s5. */
    public s5: number;

    /** uvec14 s6. */
    public s6: number;

    /** uvec14 s7. */
    public s7: number;

    /** uvec14 s8. */
    public s8: number;

    /** uvec14 s9. */
    public s9: number;

    /** uvec14 sa. */
    public sa: number;

    /** uvec14 sb. */
    public sb: number;

    /** uvec14 sc. */
    public sc: number;

    /** uvec14 sd. */
    public sd: number;

    /**
     * Encodes the specified uvec14 message. Does not implicitly {@link uvec14.verify|verify} messages.
     * @param message uvec14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec14;
}

/** Properties of a vec15. */
export interface Ivec15 {

    /** vec15 s0 */
    s0?: (number|null);

    /** vec15 s1 */
    s1?: (number|null);

    /** vec15 s2 */
    s2?: (number|null);

    /** vec15 s3 */
    s3?: (number|null);

    /** vec15 s4 */
    s4?: (number|null);

    /** vec15 s5 */
    s5?: (number|null);

    /** vec15 s6 */
    s6?: (number|null);

    /** vec15 s7 */
    s7?: (number|null);

    /** vec15 s8 */
    s8?: (number|null);

    /** vec15 s9 */
    s9?: (number|null);

    /** vec15 sa */
    sa?: (number|null);

    /** vec15 sb */
    sb?: (number|null);

    /** vec15 sc */
    sc?: (number|null);

    /** vec15 sd */
    sd?: (number|null);

    /** vec15 se */
    se?: (number|null);
}

/** Represents a vec15. */
export class vec15 implements Ivec15 {

    /**
     * Constructs a new vec15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec15);

    /** vec15 s0. */
    public s0: number;

    /** vec15 s1. */
    public s1: number;

    /** vec15 s2. */
    public s2: number;

    /** vec15 s3. */
    public s3: number;

    /** vec15 s4. */
    public s4: number;

    /** vec15 s5. */
    public s5: number;

    /** vec15 s6. */
    public s6: number;

    /** vec15 s7. */
    public s7: number;

    /** vec15 s8. */
    public s8: number;

    /** vec15 s9. */
    public s9: number;

    /** vec15 sa. */
    public sa: number;

    /** vec15 sb. */
    public sb: number;

    /** vec15 sc. */
    public sc: number;

    /** vec15 sd. */
    public sd: number;

    /** vec15 se. */
    public se: number;

    /**
     * Encodes the specified vec15 message. Does not implicitly {@link vec15.verify|verify} messages.
     * @param message vec15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec15;
}

/** Properties of a fvec15. */
export interface Ifvec15 {

    /** fvec15 s0 */
    s0?: (number|null);

    /** fvec15 s1 */
    s1?: (number|null);

    /** fvec15 s2 */
    s2?: (number|null);

    /** fvec15 s3 */
    s3?: (number|null);

    /** fvec15 s4 */
    s4?: (number|null);

    /** fvec15 s5 */
    s5?: (number|null);

    /** fvec15 s6 */
    s6?: (number|null);

    /** fvec15 s7 */
    s7?: (number|null);

    /** fvec15 s8 */
    s8?: (number|null);

    /** fvec15 s9 */
    s9?: (number|null);

    /** fvec15 sa */
    sa?: (number|null);

    /** fvec15 sb */
    sb?: (number|null);

    /** fvec15 sc */
    sc?: (number|null);

    /** fvec15 sd */
    sd?: (number|null);

    /** fvec15 se */
    se?: (number|null);
}

/** Represents a fvec15. */
export class fvec15 implements Ifvec15 {

    /**
     * Constructs a new fvec15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec15);

    /** fvec15 s0. */
    public s0: number;

    /** fvec15 s1. */
    public s1: number;

    /** fvec15 s2. */
    public s2: number;

    /** fvec15 s3. */
    public s3: number;

    /** fvec15 s4. */
    public s4: number;

    /** fvec15 s5. */
    public s5: number;

    /** fvec15 s6. */
    public s6: number;

    /** fvec15 s7. */
    public s7: number;

    /** fvec15 s8. */
    public s8: number;

    /** fvec15 s9. */
    public s9: number;

    /** fvec15 sa. */
    public sa: number;

    /** fvec15 sb. */
    public sb: number;

    /** fvec15 sc. */
    public sc: number;

    /** fvec15 sd. */
    public sd: number;

    /** fvec15 se. */
    public se: number;

    /**
     * Encodes the specified fvec15 message. Does not implicitly {@link fvec15.verify|verify} messages.
     * @param message fvec15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec15;
}

/** Properties of an ivec15. */
export interface Iivec15 {

    /** ivec15 s0 */
    s0?: (number|null);

    /** ivec15 s1 */
    s1?: (number|null);

    /** ivec15 s2 */
    s2?: (number|null);

    /** ivec15 s3 */
    s3?: (number|null);

    /** ivec15 s4 */
    s4?: (number|null);

    /** ivec15 s5 */
    s5?: (number|null);

    /** ivec15 s6 */
    s6?: (number|null);

    /** ivec15 s7 */
    s7?: (number|null);

    /** ivec15 s8 */
    s8?: (number|null);

    /** ivec15 s9 */
    s9?: (number|null);

    /** ivec15 sa */
    sa?: (number|null);

    /** ivec15 sb */
    sb?: (number|null);

    /** ivec15 sc */
    sc?: (number|null);

    /** ivec15 sd */
    sd?: (number|null);

    /** ivec15 se */
    se?: (number|null);
}

/** Represents an ivec15. */
export class ivec15 implements Iivec15 {

    /**
     * Constructs a new ivec15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec15);

    /** ivec15 s0. */
    public s0: number;

    /** ivec15 s1. */
    public s1: number;

    /** ivec15 s2. */
    public s2: number;

    /** ivec15 s3. */
    public s3: number;

    /** ivec15 s4. */
    public s4: number;

    /** ivec15 s5. */
    public s5: number;

    /** ivec15 s6. */
    public s6: number;

    /** ivec15 s7. */
    public s7: number;

    /** ivec15 s8. */
    public s8: number;

    /** ivec15 s9. */
    public s9: number;

    /** ivec15 sa. */
    public sa: number;

    /** ivec15 sb. */
    public sb: number;

    /** ivec15 sc. */
    public sc: number;

    /** ivec15 sd. */
    public sd: number;

    /** ivec15 se. */
    public se: number;

    /**
     * Encodes the specified ivec15 message. Does not implicitly {@link ivec15.verify|verify} messages.
     * @param message ivec15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec15;
}

/** Properties of an uvec15. */
export interface Iuvec15 {

    /** uvec15 s0 */
    s0?: (number|null);

    /** uvec15 s1 */
    s1?: (number|null);

    /** uvec15 s2 */
    s2?: (number|null);

    /** uvec15 s3 */
    s3?: (number|null);

    /** uvec15 s4 */
    s4?: (number|null);

    /** uvec15 s5 */
    s5?: (number|null);

    /** uvec15 s6 */
    s6?: (number|null);

    /** uvec15 s7 */
    s7?: (number|null);

    /** uvec15 s8 */
    s8?: (number|null);

    /** uvec15 s9 */
    s9?: (number|null);

    /** uvec15 sa */
    sa?: (number|null);

    /** uvec15 sb */
    sb?: (number|null);

    /** uvec15 sc */
    sc?: (number|null);

    /** uvec15 sd */
    sd?: (number|null);

    /** uvec15 se */
    se?: (number|null);
}

/** Represents an uvec15. */
export class uvec15 implements Iuvec15 {

    /**
     * Constructs a new uvec15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec15);

    /** uvec15 s0. */
    public s0: number;

    /** uvec15 s1. */
    public s1: number;

    /** uvec15 s2. */
    public s2: number;

    /** uvec15 s3. */
    public s3: number;

    /** uvec15 s4. */
    public s4: number;

    /** uvec15 s5. */
    public s5: number;

    /** uvec15 s6. */
    public s6: number;

    /** uvec15 s7. */
    public s7: number;

    /** uvec15 s8. */
    public s8: number;

    /** uvec15 s9. */
    public s9: number;

    /** uvec15 sa. */
    public sa: number;

    /** uvec15 sb. */
    public sb: number;

    /** uvec15 sc. */
    public sc: number;

    /** uvec15 sd. */
    public sd: number;

    /** uvec15 se. */
    public se: number;

    /**
     * Encodes the specified uvec15 message. Does not implicitly {@link uvec15.verify|verify} messages.
     * @param message uvec15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec15;
}

/** Properties of a vec16. */
export interface Ivec16 {

    /** vec16 s0 */
    s0?: (number|null);

    /** vec16 s1 */
    s1?: (number|null);

    /** vec16 s2 */
    s2?: (number|null);

    /** vec16 s3 */
    s3?: (number|null);

    /** vec16 s4 */
    s4?: (number|null);

    /** vec16 s5 */
    s5?: (number|null);

    /** vec16 s6 */
    s6?: (number|null);

    /** vec16 s7 */
    s7?: (number|null);

    /** vec16 s8 */
    s8?: (number|null);

    /** vec16 s9 */
    s9?: (number|null);

    /** vec16 sa */
    sa?: (number|null);

    /** vec16 sb */
    sb?: (number|null);

    /** vec16 sc */
    sc?: (number|null);

    /** vec16 sd */
    sd?: (number|null);

    /** vec16 se */
    se?: (number|null);

    /** vec16 sf */
    sf?: (number|null);
}

/** Represents a vec16. */
export class vec16 implements Ivec16 {

    /**
     * Constructs a new vec16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ivec16);

    /** vec16 s0. */
    public s0: number;

    /** vec16 s1. */
    public s1: number;

    /** vec16 s2. */
    public s2: number;

    /** vec16 s3. */
    public s3: number;

    /** vec16 s4. */
    public s4: number;

    /** vec16 s5. */
    public s5: number;

    /** vec16 s6. */
    public s6: number;

    /** vec16 s7. */
    public s7: number;

    /** vec16 s8. */
    public s8: number;

    /** vec16 s9. */
    public s9: number;

    /** vec16 sa. */
    public sa: number;

    /** vec16 sb. */
    public sb: number;

    /** vec16 sc. */
    public sc: number;

    /** vec16 sd. */
    public sd: number;

    /** vec16 se. */
    public se: number;

    /** vec16 sf. */
    public sf: number;

    /**
     * Encodes the specified vec16 message. Does not implicitly {@link vec16.verify|verify} messages.
     * @param message vec16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ivec16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a vec16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns vec16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): vec16;
}

/** Properties of a fvec16. */
export interface Ifvec16 {

    /** fvec16 s0 */
    s0?: (number|null);

    /** fvec16 s1 */
    s1?: (number|null);

    /** fvec16 s2 */
    s2?: (number|null);

    /** fvec16 s3 */
    s3?: (number|null);

    /** fvec16 s4 */
    s4?: (number|null);

    /** fvec16 s5 */
    s5?: (number|null);

    /** fvec16 s6 */
    s6?: (number|null);

    /** fvec16 s7 */
    s7?: (number|null);

    /** fvec16 s8 */
    s8?: (number|null);

    /** fvec16 s9 */
    s9?: (number|null);

    /** fvec16 sa */
    sa?: (number|null);

    /** fvec16 sb */
    sb?: (number|null);

    /** fvec16 sc */
    sc?: (number|null);

    /** fvec16 sd */
    sd?: (number|null);

    /** fvec16 se */
    se?: (number|null);

    /** fvec16 sf */
    sf?: (number|null);
}

/** Represents a fvec16. */
export class fvec16 implements Ifvec16 {

    /**
     * Constructs a new fvec16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifvec16);

    /** fvec16 s0. */
    public s0: number;

    /** fvec16 s1. */
    public s1: number;

    /** fvec16 s2. */
    public s2: number;

    /** fvec16 s3. */
    public s3: number;

    /** fvec16 s4. */
    public s4: number;

    /** fvec16 s5. */
    public s5: number;

    /** fvec16 s6. */
    public s6: number;

    /** fvec16 s7. */
    public s7: number;

    /** fvec16 s8. */
    public s8: number;

    /** fvec16 s9. */
    public s9: number;

    /** fvec16 sa. */
    public sa: number;

    /** fvec16 sb. */
    public sb: number;

    /** fvec16 sc. */
    public sc: number;

    /** fvec16 sd. */
    public sd: number;

    /** fvec16 se. */
    public se: number;

    /** fvec16 sf. */
    public sf: number;

    /**
     * Encodes the specified fvec16 message. Does not implicitly {@link fvec16.verify|verify} messages.
     * @param message fvec16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifvec16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fvec16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fvec16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fvec16;
}

/** Properties of an ivec16. */
export interface Iivec16 {

    /** ivec16 s0 */
    s0?: (number|null);

    /** ivec16 s1 */
    s1?: (number|null);

    /** ivec16 s2 */
    s2?: (number|null);

    /** ivec16 s3 */
    s3?: (number|null);

    /** ivec16 s4 */
    s4?: (number|null);

    /** ivec16 s5 */
    s5?: (number|null);

    /** ivec16 s6 */
    s6?: (number|null);

    /** ivec16 s7 */
    s7?: (number|null);

    /** ivec16 s8 */
    s8?: (number|null);

    /** ivec16 s9 */
    s9?: (number|null);

    /** ivec16 sa */
    sa?: (number|null);

    /** ivec16 sb */
    sb?: (number|null);

    /** ivec16 sc */
    sc?: (number|null);

    /** ivec16 sd */
    sd?: (number|null);

    /** ivec16 se */
    se?: (number|null);

    /** ivec16 sf */
    sf?: (number|null);
}

/** Represents an ivec16. */
export class ivec16 implements Iivec16 {

    /**
     * Constructs a new ivec16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iivec16);

    /** ivec16 s0. */
    public s0: number;

    /** ivec16 s1. */
    public s1: number;

    /** ivec16 s2. */
    public s2: number;

    /** ivec16 s3. */
    public s3: number;

    /** ivec16 s4. */
    public s4: number;

    /** ivec16 s5. */
    public s5: number;

    /** ivec16 s6. */
    public s6: number;

    /** ivec16 s7. */
    public s7: number;

    /** ivec16 s8. */
    public s8: number;

    /** ivec16 s9. */
    public s9: number;

    /** ivec16 sa. */
    public sa: number;

    /** ivec16 sb. */
    public sb: number;

    /** ivec16 sc. */
    public sc: number;

    /** ivec16 sd. */
    public sd: number;

    /** ivec16 se. */
    public se: number;

    /** ivec16 sf. */
    public sf: number;

    /**
     * Encodes the specified ivec16 message. Does not implicitly {@link ivec16.verify|verify} messages.
     * @param message ivec16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iivec16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an ivec16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns ivec16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): ivec16;
}

/** Properties of an uvec16. */
export interface Iuvec16 {

    /** uvec16 s0 */
    s0?: (number|null);

    /** uvec16 s1 */
    s1?: (number|null);

    /** uvec16 s2 */
    s2?: (number|null);

    /** uvec16 s3 */
    s3?: (number|null);

    /** uvec16 s4 */
    s4?: (number|null);

    /** uvec16 s5 */
    s5?: (number|null);

    /** uvec16 s6 */
    s6?: (number|null);

    /** uvec16 s7 */
    s7?: (number|null);

    /** uvec16 s8 */
    s8?: (number|null);

    /** uvec16 s9 */
    s9?: (number|null);

    /** uvec16 sa */
    sa?: (number|null);

    /** uvec16 sb */
    sb?: (number|null);

    /** uvec16 sc */
    sc?: (number|null);

    /** uvec16 sd */
    sd?: (number|null);

    /** uvec16 se */
    se?: (number|null);

    /** uvec16 sf */
    sf?: (number|null);
}

/** Represents an uvec16. */
export class uvec16 implements Iuvec16 {

    /**
     * Constructs a new uvec16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iuvec16);

    /** uvec16 s0. */
    public s0: number;

    /** uvec16 s1. */
    public s1: number;

    /** uvec16 s2. */
    public s2: number;

    /** uvec16 s3. */
    public s3: number;

    /** uvec16 s4. */
    public s4: number;

    /** uvec16 s5. */
    public s5: number;

    /** uvec16 s6. */
    public s6: number;

    /** uvec16 s7. */
    public s7: number;

    /** uvec16 s8. */
    public s8: number;

    /** uvec16 s9. */
    public s9: number;

    /** uvec16 sa. */
    public sa: number;

    /** uvec16 sb. */
    public sb: number;

    /** uvec16 sc. */
    public sc: number;

    /** uvec16 sd. */
    public sd: number;

    /** uvec16 se. */
    public se: number;

    /** uvec16 sf. */
    public sf: number;

    /**
     * Encodes the specified uvec16 message. Does not implicitly {@link uvec16.verify|verify} messages.
     * @param message uvec16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iuvec16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an uvec16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns uvec16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): uvec16;
}

/** Properties of a mat. */
export interface Imat {

    /** mat rows */
    rows?: (number|null);

    /** mat cols */
    cols?: (number|null);

    /** mat v */
    v?: (number[]|null);
}

/** Represents a mat. */
export class mat implements Imat {

    /**
     * Constructs a new mat.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat);

    /** mat rows. */
    public rows: number;

    /** mat cols. */
    public cols: number;

    /** mat v. */
    public v: number[];

    /**
     * Encodes the specified mat message. Does not implicitly {@link mat.verify|verify} messages.
     * @param message mat message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat;
}

/** Properties of a fmat. */
export interface Ifmat {

    /** fmat rows */
    rows?: (number|null);

    /** fmat cols */
    cols?: (number|null);

    /** fmat v */
    v?: (number[]|null);
}

/** Represents a fmat. */
export class fmat implements Ifmat {

    /**
     * Constructs a new fmat.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat);

    /** fmat rows. */
    public rows: number;

    /** fmat cols. */
    public cols: number;

    /** fmat v. */
    public v: number[];

    /**
     * Encodes the specified fmat message. Does not implicitly {@link fmat.verify|verify} messages.
     * @param message fmat message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat;
}

/** Properties of an imat. */
export interface Iimat {

    /** imat rows */
    rows?: (number|null);

    /** imat cols */
    cols?: (number|null);

    /** imat v */
    v?: (number[]|null);
}

/** Represents an imat. */
export class imat implements Iimat {

    /**
     * Constructs a new imat.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat);

    /** imat rows. */
    public rows: number;

    /** imat cols. */
    public cols: number;

    /** imat v. */
    public v: number[];

    /**
     * Encodes the specified imat message. Does not implicitly {@link imat.verify|verify} messages.
     * @param message imat message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat;
}

/** Properties of an umat. */
export interface Iumat {

    /** umat rows */
    rows?: (number|null);

    /** umat cols */
    cols?: (number|null);

    /** umat v */
    v?: (number[]|null);
}

/** Represents an umat. */
export class umat implements Iumat {

    /**
     * Constructs a new umat.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat);

    /** umat rows. */
    public rows: number;

    /** umat cols. */
    public cols: number;

    /** umat v. */
    public v: number[];

    /**
     * Encodes the specified umat message. Does not implicitly {@link umat.verify|verify} messages.
     * @param message umat message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat;
}

/** Properties of a cmat. */
export interface Icmat {

    /** cmat rows */
    rows?: (number|null);

    /** cmat cols */
    cols?: (number|null);

    /** cmat v */
    v?: (Uint8Array|null);
}

/** Represents a cmat. */
export class cmat implements Icmat {

    /**
     * Constructs a new cmat.
     * @param [properties] Properties to set
     */
    constructor(properties?: Icmat);

    /** cmat rows. */
    public rows: number;

    /** cmat cols. */
    public cols: number;

    /** cmat v. */
    public v: Uint8Array;

    /**
     * Encodes the specified cmat message. Does not implicitly {@link cmat.verify|verify} messages.
     * @param message cmat message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Icmat, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a cmat message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns cmat
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): cmat;
}

/** Properties of a mat2. */
export interface Imat2 {

    /** mat2 x */
    x?: (Ivec2|null);

    /** mat2 y */
    y?: (Ivec2|null);
}

/** Represents a mat2. */
export class mat2 implements Imat2 {

    /**
     * Constructs a new mat2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat2);

    /** mat2 x. */
    public x?: (Ivec2|null);

    /** mat2 y. */
    public y?: (Ivec2|null);

    /**
     * Encodes the specified mat2 message. Does not implicitly {@link mat2.verify|verify} messages.
     * @param message mat2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat2;
}

/** Properties of a fmat2. */
export interface Ifmat2 {

    /** fmat2 x */
    x?: (Ifvec2|null);

    /** fmat2 y */
    y?: (Ifvec2|null);
}

/** Represents a fmat2. */
export class fmat2 implements Ifmat2 {

    /**
     * Constructs a new fmat2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat2);

    /** fmat2 x. */
    public x?: (Ifvec2|null);

    /** fmat2 y. */
    public y?: (Ifvec2|null);

    /**
     * Encodes the specified fmat2 message. Does not implicitly {@link fmat2.verify|verify} messages.
     * @param message fmat2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat2;
}

/** Properties of an imat2. */
export interface Iimat2 {

    /** imat2 x */
    x?: (Iivec2|null);

    /** imat2 y */
    y?: (Iivec2|null);
}

/** Represents an imat2. */
export class imat2 implements Iimat2 {

    /**
     * Constructs a new imat2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat2);

    /** imat2 x. */
    public x?: (Iivec2|null);

    /** imat2 y. */
    public y?: (Iivec2|null);

    /**
     * Encodes the specified imat2 message. Does not implicitly {@link imat2.verify|verify} messages.
     * @param message imat2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat2;
}

/** Properties of an umat2. */
export interface Iumat2 {

    /** umat2 x */
    x?: (Iuvec2|null);

    /** umat2 y */
    y?: (Iuvec2|null);
}

/** Represents an umat2. */
export class umat2 implements Iumat2 {

    /**
     * Constructs a new umat2.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat2);

    /** umat2 x. */
    public x?: (Iuvec2|null);

    /** umat2 y. */
    public y?: (Iuvec2|null);

    /**
     * Encodes the specified umat2 message. Does not implicitly {@link umat2.verify|verify} messages.
     * @param message umat2 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat2, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat2 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat2
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat2;
}

/** Properties of a mat3. */
export interface Imat3 {

    /** mat3 x */
    x?: (Ivec3|null);

    /** mat3 y */
    y?: (Ivec3|null);

    /** mat3 z */
    z?: (Ivec3|null);
}

/** Represents a mat3. */
export class mat3 implements Imat3 {

    /**
     * Constructs a new mat3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat3);

    /** mat3 x. */
    public x?: (Ivec3|null);

    /** mat3 y. */
    public y?: (Ivec3|null);

    /** mat3 z. */
    public z?: (Ivec3|null);

    /**
     * Encodes the specified mat3 message. Does not implicitly {@link mat3.verify|verify} messages.
     * @param message mat3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat3;
}

/** Properties of a fmat3. */
export interface Ifmat3 {

    /** fmat3 x */
    x?: (Ifvec3|null);

    /** fmat3 y */
    y?: (Ifvec3|null);

    /** fmat3 z */
    z?: (Ifvec3|null);
}

/** Represents a fmat3. */
export class fmat3 implements Ifmat3 {

    /**
     * Constructs a new fmat3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat3);

    /** fmat3 x. */
    public x?: (Ifvec3|null);

    /** fmat3 y. */
    public y?: (Ifvec3|null);

    /** fmat3 z. */
    public z?: (Ifvec3|null);

    /**
     * Encodes the specified fmat3 message. Does not implicitly {@link fmat3.verify|verify} messages.
     * @param message fmat3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat3;
}

/** Properties of an imat3. */
export interface Iimat3 {

    /** imat3 x */
    x?: (Iivec3|null);

    /** imat3 y */
    y?: (Iivec3|null);

    /** imat3 z */
    z?: (Iivec3|null);
}

/** Represents an imat3. */
export class imat3 implements Iimat3 {

    /**
     * Constructs a new imat3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat3);

    /** imat3 x. */
    public x?: (Iivec3|null);

    /** imat3 y. */
    public y?: (Iivec3|null);

    /** imat3 z. */
    public z?: (Iivec3|null);

    /**
     * Encodes the specified imat3 message. Does not implicitly {@link imat3.verify|verify} messages.
     * @param message imat3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat3;
}

/** Properties of an umat3. */
export interface Iumat3 {

    /** umat3 x */
    x?: (Iuvec3|null);

    /** umat3 y */
    y?: (Iuvec3|null);

    /** umat3 z */
    z?: (Iuvec3|null);
}

/** Represents an umat3. */
export class umat3 implements Iumat3 {

    /**
     * Constructs a new umat3.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat3);

    /** umat3 x. */
    public x?: (Iuvec3|null);

    /** umat3 y. */
    public y?: (Iuvec3|null);

    /** umat3 z. */
    public z?: (Iuvec3|null);

    /**
     * Encodes the specified umat3 message. Does not implicitly {@link umat3.verify|verify} messages.
     * @param message umat3 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat3, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat3 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat3
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat3;
}

/** Properties of a mat4. */
export interface Imat4 {

    /** mat4 x */
    x?: (Ivec4|null);

    /** mat4 y */
    y?: (Ivec4|null);

    /** mat4 z */
    z?: (Ivec4|null);

    /** mat4 t */
    t?: (Ivec4|null);
}

/** Represents a mat4. */
export class mat4 implements Imat4 {

    /**
     * Constructs a new mat4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat4);

    /** mat4 x. */
    public x?: (Ivec4|null);

    /** mat4 y. */
    public y?: (Ivec4|null);

    /** mat4 z. */
    public z?: (Ivec4|null);

    /** mat4 t. */
    public t?: (Ivec4|null);

    /**
     * Encodes the specified mat4 message. Does not implicitly {@link mat4.verify|verify} messages.
     * @param message mat4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat4;
}

/** Properties of a fmat4. */
export interface Ifmat4 {

    /** fmat4 x */
    x?: (Ifvec4|null);

    /** fmat4 y */
    y?: (Ifvec4|null);

    /** fmat4 z */
    z?: (Ifvec4|null);

    /** fmat4 t */
    t?: (Ifvec4|null);
}

/** Represents a fmat4. */
export class fmat4 implements Ifmat4 {

    /**
     * Constructs a new fmat4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat4);

    /** fmat4 x. */
    public x?: (Ifvec4|null);

    /** fmat4 y. */
    public y?: (Ifvec4|null);

    /** fmat4 z. */
    public z?: (Ifvec4|null);

    /** fmat4 t. */
    public t?: (Ifvec4|null);

    /**
     * Encodes the specified fmat4 message. Does not implicitly {@link fmat4.verify|verify} messages.
     * @param message fmat4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat4;
}

/** Properties of an imat4. */
export interface Iimat4 {

    /** imat4 x */
    x?: (Iivec4|null);

    /** imat4 y */
    y?: (Iivec4|null);

    /** imat4 z */
    z?: (Iivec4|null);

    /** imat4 t */
    t?: (Iivec4|null);
}

/** Represents an imat4. */
export class imat4 implements Iimat4 {

    /**
     * Constructs a new imat4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat4);

    /** imat4 x. */
    public x?: (Iivec4|null);

    /** imat4 y. */
    public y?: (Iivec4|null);

    /** imat4 z. */
    public z?: (Iivec4|null);

    /** imat4 t. */
    public t?: (Iivec4|null);

    /**
     * Encodes the specified imat4 message. Does not implicitly {@link imat4.verify|verify} messages.
     * @param message imat4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat4;
}

/** Properties of an umat4. */
export interface Iumat4 {

    /** umat4 x */
    x?: (Iuvec4|null);

    /** umat4 y */
    y?: (Iuvec4|null);

    /** umat4 z */
    z?: (Iuvec4|null);

    /** umat4 t */
    t?: (Iuvec4|null);
}

/** Represents an umat4. */
export class umat4 implements Iumat4 {

    /**
     * Constructs a new umat4.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat4);

    /** umat4 x. */
    public x?: (Iuvec4|null);

    /** umat4 y. */
    public y?: (Iuvec4|null);

    /** umat4 z. */
    public z?: (Iuvec4|null);

    /** umat4 t. */
    public t?: (Iuvec4|null);

    /**
     * Encodes the specified umat4 message. Does not implicitly {@link umat4.verify|verify} messages.
     * @param message umat4 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat4, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat4 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat4
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat4;
}

/** Properties of a mat5. */
export interface Imat5 {

    /** mat5 s0 */
    s0?: (Ivec5|null);

    /** mat5 s1 */
    s1?: (Ivec5|null);

    /** mat5 s2 */
    s2?: (Ivec5|null);

    /** mat5 s3 */
    s3?: (Ivec5|null);

    /** mat5 s4 */
    s4?: (Ivec5|null);
}

/** Represents a mat5. */
export class mat5 implements Imat5 {

    /**
     * Constructs a new mat5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat5);

    /** mat5 s0. */
    public s0?: (Ivec5|null);

    /** mat5 s1. */
    public s1?: (Ivec5|null);

    /** mat5 s2. */
    public s2?: (Ivec5|null);

    /** mat5 s3. */
    public s3?: (Ivec5|null);

    /** mat5 s4. */
    public s4?: (Ivec5|null);

    /**
     * Encodes the specified mat5 message. Does not implicitly {@link mat5.verify|verify} messages.
     * @param message mat5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat5;
}

/** Properties of a fmat5. */
export interface Ifmat5 {

    /** fmat5 s0 */
    s0?: (Ifvec5|null);

    /** fmat5 s1 */
    s1?: (Ifvec5|null);

    /** fmat5 s2 */
    s2?: (Ifvec5|null);

    /** fmat5 s3 */
    s3?: (Ifvec5|null);

    /** fmat5 s4 */
    s4?: (Ifvec5|null);
}

/** Represents a fmat5. */
export class fmat5 implements Ifmat5 {

    /**
     * Constructs a new fmat5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat5);

    /** fmat5 s0. */
    public s0?: (Ifvec5|null);

    /** fmat5 s1. */
    public s1?: (Ifvec5|null);

    /** fmat5 s2. */
    public s2?: (Ifvec5|null);

    /** fmat5 s3. */
    public s3?: (Ifvec5|null);

    /** fmat5 s4. */
    public s4?: (Ifvec5|null);

    /**
     * Encodes the specified fmat5 message. Does not implicitly {@link fmat5.verify|verify} messages.
     * @param message fmat5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat5;
}

/** Properties of an imat5. */
export interface Iimat5 {

    /** imat5 s0 */
    s0?: (Iivec5|null);

    /** imat5 s1 */
    s1?: (Iivec5|null);

    /** imat5 s2 */
    s2?: (Iivec5|null);

    /** imat5 s3 */
    s3?: (Iivec5|null);

    /** imat5 s4 */
    s4?: (Iivec5|null);
}

/** Represents an imat5. */
export class imat5 implements Iimat5 {

    /**
     * Constructs a new imat5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat5);

    /** imat5 s0. */
    public s0?: (Iivec5|null);

    /** imat5 s1. */
    public s1?: (Iivec5|null);

    /** imat5 s2. */
    public s2?: (Iivec5|null);

    /** imat5 s3. */
    public s3?: (Iivec5|null);

    /** imat5 s4. */
    public s4?: (Iivec5|null);

    /**
     * Encodes the specified imat5 message. Does not implicitly {@link imat5.verify|verify} messages.
     * @param message imat5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat5;
}

/** Properties of an umat5. */
export interface Iumat5 {

    /** umat5 s0 */
    s0?: (Iuvec5|null);

    /** umat5 s1 */
    s1?: (Iuvec5|null);

    /** umat5 s2 */
    s2?: (Iuvec5|null);

    /** umat5 s3 */
    s3?: (Iuvec5|null);

    /** umat5 s4 */
    s4?: (Iuvec5|null);
}

/** Represents an umat5. */
export class umat5 implements Iumat5 {

    /**
     * Constructs a new umat5.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat5);

    /** umat5 s0. */
    public s0?: (Iuvec5|null);

    /** umat5 s1. */
    public s1?: (Iuvec5|null);

    /** umat5 s2. */
    public s2?: (Iuvec5|null);

    /** umat5 s3. */
    public s3?: (Iuvec5|null);

    /** umat5 s4. */
    public s4?: (Iuvec5|null);

    /**
     * Encodes the specified umat5 message. Does not implicitly {@link umat5.verify|verify} messages.
     * @param message umat5 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat5, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat5 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat5
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat5;
}

/** Properties of a mat6. */
export interface Imat6 {

    /** mat6 s0 */
    s0?: (Ivec6|null);

    /** mat6 s1 */
    s1?: (Ivec6|null);

    /** mat6 s2 */
    s2?: (Ivec6|null);

    /** mat6 s3 */
    s3?: (Ivec6|null);

    /** mat6 s4 */
    s4?: (Ivec6|null);

    /** mat6 s5 */
    s5?: (Ivec6|null);
}

/** Represents a mat6. */
export class mat6 implements Imat6 {

    /**
     * Constructs a new mat6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat6);

    /** mat6 s0. */
    public s0?: (Ivec6|null);

    /** mat6 s1. */
    public s1?: (Ivec6|null);

    /** mat6 s2. */
    public s2?: (Ivec6|null);

    /** mat6 s3. */
    public s3?: (Ivec6|null);

    /** mat6 s4. */
    public s4?: (Ivec6|null);

    /** mat6 s5. */
    public s5?: (Ivec6|null);

    /**
     * Encodes the specified mat6 message. Does not implicitly {@link mat6.verify|verify} messages.
     * @param message mat6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat6;
}

/** Properties of a fmat6. */
export interface Ifmat6 {

    /** fmat6 s0 */
    s0?: (Ifvec6|null);

    /** fmat6 s1 */
    s1?: (Ifvec6|null);

    /** fmat6 s2 */
    s2?: (Ifvec6|null);

    /** fmat6 s3 */
    s3?: (Ifvec6|null);

    /** fmat6 s4 */
    s4?: (Ifvec6|null);

    /** fmat6 s5 */
    s5?: (Ifvec6|null);
}

/** Represents a fmat6. */
export class fmat6 implements Ifmat6 {

    /**
     * Constructs a new fmat6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat6);

    /** fmat6 s0. */
    public s0?: (Ifvec6|null);

    /** fmat6 s1. */
    public s1?: (Ifvec6|null);

    /** fmat6 s2. */
    public s2?: (Ifvec6|null);

    /** fmat6 s3. */
    public s3?: (Ifvec6|null);

    /** fmat6 s4. */
    public s4?: (Ifvec6|null);

    /** fmat6 s5. */
    public s5?: (Ifvec6|null);

    /**
     * Encodes the specified fmat6 message. Does not implicitly {@link fmat6.verify|verify} messages.
     * @param message fmat6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat6;
}

/** Properties of an imat6. */
export interface Iimat6 {

    /** imat6 s0 */
    s0?: (Iivec6|null);

    /** imat6 s1 */
    s1?: (Iivec6|null);

    /** imat6 s2 */
    s2?: (Iivec6|null);

    /** imat6 s3 */
    s3?: (Iivec6|null);

    /** imat6 s4 */
    s4?: (Iivec6|null);

    /** imat6 s5 */
    s5?: (Iivec6|null);
}

/** Represents an imat6. */
export class imat6 implements Iimat6 {

    /**
     * Constructs a new imat6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat6);

    /** imat6 s0. */
    public s0?: (Iivec6|null);

    /** imat6 s1. */
    public s1?: (Iivec6|null);

    /** imat6 s2. */
    public s2?: (Iivec6|null);

    /** imat6 s3. */
    public s3?: (Iivec6|null);

    /** imat6 s4. */
    public s4?: (Iivec6|null);

    /** imat6 s5. */
    public s5?: (Iivec6|null);

    /**
     * Encodes the specified imat6 message. Does not implicitly {@link imat6.verify|verify} messages.
     * @param message imat6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat6;
}

/** Properties of an umat6. */
export interface Iumat6 {

    /** umat6 s0 */
    s0?: (Iuvec6|null);

    /** umat6 s1 */
    s1?: (Iuvec6|null);

    /** umat6 s2 */
    s2?: (Iuvec6|null);

    /** umat6 s3 */
    s3?: (Iuvec6|null);

    /** umat6 s4 */
    s4?: (Iuvec6|null);

    /** umat6 s5 */
    s5?: (Iuvec6|null);
}

/** Represents an umat6. */
export class umat6 implements Iumat6 {

    /**
     * Constructs a new umat6.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat6);

    /** umat6 s0. */
    public s0?: (Iuvec6|null);

    /** umat6 s1. */
    public s1?: (Iuvec6|null);

    /** umat6 s2. */
    public s2?: (Iuvec6|null);

    /** umat6 s3. */
    public s3?: (Iuvec6|null);

    /** umat6 s4. */
    public s4?: (Iuvec6|null);

    /** umat6 s5. */
    public s5?: (Iuvec6|null);

    /**
     * Encodes the specified umat6 message. Does not implicitly {@link umat6.verify|verify} messages.
     * @param message umat6 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat6, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat6 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat6
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat6;
}

/** Properties of a mat7. */
export interface Imat7 {

    /** mat7 s0 */
    s0?: (Ivec7|null);

    /** mat7 s1 */
    s1?: (Ivec7|null);

    /** mat7 s2 */
    s2?: (Ivec7|null);

    /** mat7 s3 */
    s3?: (Ivec7|null);

    /** mat7 s4 */
    s4?: (Ivec7|null);

    /** mat7 s5 */
    s5?: (Ivec7|null);

    /** mat7 s6 */
    s6?: (Ivec7|null);
}

/** Represents a mat7. */
export class mat7 implements Imat7 {

    /**
     * Constructs a new mat7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat7);

    /** mat7 s0. */
    public s0?: (Ivec7|null);

    /** mat7 s1. */
    public s1?: (Ivec7|null);

    /** mat7 s2. */
    public s2?: (Ivec7|null);

    /** mat7 s3. */
    public s3?: (Ivec7|null);

    /** mat7 s4. */
    public s4?: (Ivec7|null);

    /** mat7 s5. */
    public s5?: (Ivec7|null);

    /** mat7 s6. */
    public s6?: (Ivec7|null);

    /**
     * Encodes the specified mat7 message. Does not implicitly {@link mat7.verify|verify} messages.
     * @param message mat7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat7;
}

/** Properties of a fmat7. */
export interface Ifmat7 {

    /** fmat7 s0 */
    s0?: (Ifvec7|null);

    /** fmat7 s1 */
    s1?: (Ifvec7|null);

    /** fmat7 s2 */
    s2?: (Ifvec7|null);

    /** fmat7 s3 */
    s3?: (Ifvec7|null);

    /** fmat7 s4 */
    s4?: (Ifvec7|null);

    /** fmat7 s5 */
    s5?: (Ifvec7|null);

    /** fmat7 s6 */
    s6?: (Ifvec7|null);
}

/** Represents a fmat7. */
export class fmat7 implements Ifmat7 {

    /**
     * Constructs a new fmat7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat7);

    /** fmat7 s0. */
    public s0?: (Ifvec7|null);

    /** fmat7 s1. */
    public s1?: (Ifvec7|null);

    /** fmat7 s2. */
    public s2?: (Ifvec7|null);

    /** fmat7 s3. */
    public s3?: (Ifvec7|null);

    /** fmat7 s4. */
    public s4?: (Ifvec7|null);

    /** fmat7 s5. */
    public s5?: (Ifvec7|null);

    /** fmat7 s6. */
    public s6?: (Ifvec7|null);

    /**
     * Encodes the specified fmat7 message. Does not implicitly {@link fmat7.verify|verify} messages.
     * @param message fmat7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat7;
}

/** Properties of an imat7. */
export interface Iimat7 {

    /** imat7 s0 */
    s0?: (Iivec7|null);

    /** imat7 s1 */
    s1?: (Iivec7|null);

    /** imat7 s2 */
    s2?: (Iivec7|null);

    /** imat7 s3 */
    s3?: (Iivec7|null);

    /** imat7 s4 */
    s4?: (Iivec7|null);

    /** imat7 s5 */
    s5?: (Iivec7|null);

    /** imat7 s6 */
    s6?: (Iivec7|null);
}

/** Represents an imat7. */
export class imat7 implements Iimat7 {

    /**
     * Constructs a new imat7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat7);

    /** imat7 s0. */
    public s0?: (Iivec7|null);

    /** imat7 s1. */
    public s1?: (Iivec7|null);

    /** imat7 s2. */
    public s2?: (Iivec7|null);

    /** imat7 s3. */
    public s3?: (Iivec7|null);

    /** imat7 s4. */
    public s4?: (Iivec7|null);

    /** imat7 s5. */
    public s5?: (Iivec7|null);

    /** imat7 s6. */
    public s6?: (Iivec7|null);

    /**
     * Encodes the specified imat7 message. Does not implicitly {@link imat7.verify|verify} messages.
     * @param message imat7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat7;
}

/** Properties of an umat7. */
export interface Iumat7 {

    /** umat7 s0 */
    s0?: (Iuvec7|null);

    /** umat7 s1 */
    s1?: (Iuvec7|null);

    /** umat7 s2 */
    s2?: (Iuvec7|null);

    /** umat7 s3 */
    s3?: (Iuvec7|null);

    /** umat7 s4 */
    s4?: (Iuvec7|null);

    /** umat7 s5 */
    s5?: (Iuvec7|null);

    /** umat7 s6 */
    s6?: (Iuvec7|null);
}

/** Represents an umat7. */
export class umat7 implements Iumat7 {

    /**
     * Constructs a new umat7.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat7);

    /** umat7 s0. */
    public s0?: (Iuvec7|null);

    /** umat7 s1. */
    public s1?: (Iuvec7|null);

    /** umat7 s2. */
    public s2?: (Iuvec7|null);

    /** umat7 s3. */
    public s3?: (Iuvec7|null);

    /** umat7 s4. */
    public s4?: (Iuvec7|null);

    /** umat7 s5. */
    public s5?: (Iuvec7|null);

    /** umat7 s6. */
    public s6?: (Iuvec7|null);

    /**
     * Encodes the specified umat7 message. Does not implicitly {@link umat7.verify|verify} messages.
     * @param message umat7 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat7, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat7 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat7
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat7;
}

/** Properties of a mat8. */
export interface Imat8 {

    /** mat8 s0 */
    s0?: (Ivec8|null);

    /** mat8 s1 */
    s1?: (Ivec8|null);

    /** mat8 s2 */
    s2?: (Ivec8|null);

    /** mat8 s3 */
    s3?: (Ivec8|null);

    /** mat8 s4 */
    s4?: (Ivec8|null);

    /** mat8 s5 */
    s5?: (Ivec8|null);

    /** mat8 s6 */
    s6?: (Ivec8|null);

    /** mat8 s7 */
    s7?: (Ivec8|null);
}

/** Represents a mat8. */
export class mat8 implements Imat8 {

    /**
     * Constructs a new mat8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat8);

    /** mat8 s0. */
    public s0?: (Ivec8|null);

    /** mat8 s1. */
    public s1?: (Ivec8|null);

    /** mat8 s2. */
    public s2?: (Ivec8|null);

    /** mat8 s3. */
    public s3?: (Ivec8|null);

    /** mat8 s4. */
    public s4?: (Ivec8|null);

    /** mat8 s5. */
    public s5?: (Ivec8|null);

    /** mat8 s6. */
    public s6?: (Ivec8|null);

    /** mat8 s7. */
    public s7?: (Ivec8|null);

    /**
     * Encodes the specified mat8 message. Does not implicitly {@link mat8.verify|verify} messages.
     * @param message mat8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat8;
}

/** Properties of a fmat8. */
export interface Ifmat8 {

    /** fmat8 s0 */
    s0?: (Ifvec8|null);

    /** fmat8 s1 */
    s1?: (Ifvec8|null);

    /** fmat8 s2 */
    s2?: (Ifvec8|null);

    /** fmat8 s3 */
    s3?: (Ifvec8|null);

    /** fmat8 s4 */
    s4?: (Ifvec8|null);

    /** fmat8 s5 */
    s5?: (Ifvec8|null);

    /** fmat8 s6 */
    s6?: (Ifvec8|null);

    /** fmat8 s7 */
    s7?: (Ifvec8|null);
}

/** Represents a fmat8. */
export class fmat8 implements Ifmat8 {

    /**
     * Constructs a new fmat8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat8);

    /** fmat8 s0. */
    public s0?: (Ifvec8|null);

    /** fmat8 s1. */
    public s1?: (Ifvec8|null);

    /** fmat8 s2. */
    public s2?: (Ifvec8|null);

    /** fmat8 s3. */
    public s3?: (Ifvec8|null);

    /** fmat8 s4. */
    public s4?: (Ifvec8|null);

    /** fmat8 s5. */
    public s5?: (Ifvec8|null);

    /** fmat8 s6. */
    public s6?: (Ifvec8|null);

    /** fmat8 s7. */
    public s7?: (Ifvec8|null);

    /**
     * Encodes the specified fmat8 message. Does not implicitly {@link fmat8.verify|verify} messages.
     * @param message fmat8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat8;
}

/** Properties of an imat8. */
export interface Iimat8 {

    /** imat8 s0 */
    s0?: (Iivec8|null);

    /** imat8 s1 */
    s1?: (Iivec8|null);

    /** imat8 s2 */
    s2?: (Iivec8|null);

    /** imat8 s3 */
    s3?: (Iivec8|null);

    /** imat8 s4 */
    s4?: (Iivec8|null);

    /** imat8 s5 */
    s5?: (Iivec8|null);

    /** imat8 s6 */
    s6?: (Iivec8|null);

    /** imat8 s7 */
    s7?: (Iivec8|null);
}

/** Represents an imat8. */
export class imat8 implements Iimat8 {

    /**
     * Constructs a new imat8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat8);

    /** imat8 s0. */
    public s0?: (Iivec8|null);

    /** imat8 s1. */
    public s1?: (Iivec8|null);

    /** imat8 s2. */
    public s2?: (Iivec8|null);

    /** imat8 s3. */
    public s3?: (Iivec8|null);

    /** imat8 s4. */
    public s4?: (Iivec8|null);

    /** imat8 s5. */
    public s5?: (Iivec8|null);

    /** imat8 s6. */
    public s6?: (Iivec8|null);

    /** imat8 s7. */
    public s7?: (Iivec8|null);

    /**
     * Encodes the specified imat8 message. Does not implicitly {@link imat8.verify|verify} messages.
     * @param message imat8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat8;
}

/** Properties of an umat8. */
export interface Iumat8 {

    /** umat8 s0 */
    s0?: (Iuvec8|null);

    /** umat8 s1 */
    s1?: (Iuvec8|null);

    /** umat8 s2 */
    s2?: (Iuvec8|null);

    /** umat8 s3 */
    s3?: (Iuvec8|null);

    /** umat8 s4 */
    s4?: (Iuvec8|null);

    /** umat8 s5 */
    s5?: (Iuvec8|null);

    /** umat8 s6 */
    s6?: (Iuvec8|null);

    /** umat8 s7 */
    s7?: (Iuvec8|null);
}

/** Represents an umat8. */
export class umat8 implements Iumat8 {

    /**
     * Constructs a new umat8.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat8);

    /** umat8 s0. */
    public s0?: (Iuvec8|null);

    /** umat8 s1. */
    public s1?: (Iuvec8|null);

    /** umat8 s2. */
    public s2?: (Iuvec8|null);

    /** umat8 s3. */
    public s3?: (Iuvec8|null);

    /** umat8 s4. */
    public s4?: (Iuvec8|null);

    /** umat8 s5. */
    public s5?: (Iuvec8|null);

    /** umat8 s6. */
    public s6?: (Iuvec8|null);

    /** umat8 s7. */
    public s7?: (Iuvec8|null);

    /**
     * Encodes the specified umat8 message. Does not implicitly {@link umat8.verify|verify} messages.
     * @param message umat8 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat8, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat8 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat8
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat8;
}

/** Properties of a mat9. */
export interface Imat9 {

    /** mat9 s0 */
    s0?: (Ivec9|null);

    /** mat9 s1 */
    s1?: (Ivec9|null);

    /** mat9 s2 */
    s2?: (Ivec9|null);

    /** mat9 s3 */
    s3?: (Ivec9|null);

    /** mat9 s4 */
    s4?: (Ivec9|null);

    /** mat9 s5 */
    s5?: (Ivec9|null);

    /** mat9 s6 */
    s6?: (Ivec9|null);

    /** mat9 s7 */
    s7?: (Ivec9|null);

    /** mat9 s8 */
    s8?: (Ivec9|null);
}

/** Represents a mat9. */
export class mat9 implements Imat9 {

    /**
     * Constructs a new mat9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat9);

    /** mat9 s0. */
    public s0?: (Ivec9|null);

    /** mat9 s1. */
    public s1?: (Ivec9|null);

    /** mat9 s2. */
    public s2?: (Ivec9|null);

    /** mat9 s3. */
    public s3?: (Ivec9|null);

    /** mat9 s4. */
    public s4?: (Ivec9|null);

    /** mat9 s5. */
    public s5?: (Ivec9|null);

    /** mat9 s6. */
    public s6?: (Ivec9|null);

    /** mat9 s7. */
    public s7?: (Ivec9|null);

    /** mat9 s8. */
    public s8?: (Ivec9|null);

    /**
     * Encodes the specified mat9 message. Does not implicitly {@link mat9.verify|verify} messages.
     * @param message mat9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat9;
}

/** Properties of a fmat9. */
export interface Ifmat9 {

    /** fmat9 s0 */
    s0?: (Ifvec9|null);

    /** fmat9 s1 */
    s1?: (Ifvec9|null);

    /** fmat9 s2 */
    s2?: (Ifvec9|null);

    /** fmat9 s3 */
    s3?: (Ifvec9|null);

    /** fmat9 s4 */
    s4?: (Ifvec9|null);

    /** fmat9 s5 */
    s5?: (Ifvec9|null);

    /** fmat9 s6 */
    s6?: (Ifvec9|null);

    /** fmat9 s7 */
    s7?: (Ifvec9|null);

    /** fmat9 s8 */
    s8?: (Ifvec9|null);
}

/** Represents a fmat9. */
export class fmat9 implements Ifmat9 {

    /**
     * Constructs a new fmat9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat9);

    /** fmat9 s0. */
    public s0?: (Ifvec9|null);

    /** fmat9 s1. */
    public s1?: (Ifvec9|null);

    /** fmat9 s2. */
    public s2?: (Ifvec9|null);

    /** fmat9 s3. */
    public s3?: (Ifvec9|null);

    /** fmat9 s4. */
    public s4?: (Ifvec9|null);

    /** fmat9 s5. */
    public s5?: (Ifvec9|null);

    /** fmat9 s6. */
    public s6?: (Ifvec9|null);

    /** fmat9 s7. */
    public s7?: (Ifvec9|null);

    /** fmat9 s8. */
    public s8?: (Ifvec9|null);

    /**
     * Encodes the specified fmat9 message. Does not implicitly {@link fmat9.verify|verify} messages.
     * @param message fmat9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat9;
}

/** Properties of an imat9. */
export interface Iimat9 {

    /** imat9 s0 */
    s0?: (Iivec9|null);

    /** imat9 s1 */
    s1?: (Iivec9|null);

    /** imat9 s2 */
    s2?: (Iivec9|null);

    /** imat9 s3 */
    s3?: (Iivec9|null);

    /** imat9 s4 */
    s4?: (Iivec9|null);

    /** imat9 s5 */
    s5?: (Iivec9|null);

    /** imat9 s6 */
    s6?: (Iivec9|null);

    /** imat9 s7 */
    s7?: (Iivec9|null);

    /** imat9 s8 */
    s8?: (Iivec9|null);
}

/** Represents an imat9. */
export class imat9 implements Iimat9 {

    /**
     * Constructs a new imat9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat9);

    /** imat9 s0. */
    public s0?: (Iivec9|null);

    /** imat9 s1. */
    public s1?: (Iivec9|null);

    /** imat9 s2. */
    public s2?: (Iivec9|null);

    /** imat9 s3. */
    public s3?: (Iivec9|null);

    /** imat9 s4. */
    public s4?: (Iivec9|null);

    /** imat9 s5. */
    public s5?: (Iivec9|null);

    /** imat9 s6. */
    public s6?: (Iivec9|null);

    /** imat9 s7. */
    public s7?: (Iivec9|null);

    /** imat9 s8. */
    public s8?: (Iivec9|null);

    /**
     * Encodes the specified imat9 message. Does not implicitly {@link imat9.verify|verify} messages.
     * @param message imat9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat9;
}

/** Properties of an umat9. */
export interface Iumat9 {

    /** umat9 s0 */
    s0?: (Iuvec9|null);

    /** umat9 s1 */
    s1?: (Iuvec9|null);

    /** umat9 s2 */
    s2?: (Iuvec9|null);

    /** umat9 s3 */
    s3?: (Iuvec9|null);

    /** umat9 s4 */
    s4?: (Iuvec9|null);

    /** umat9 s5 */
    s5?: (Iuvec9|null);

    /** umat9 s6 */
    s6?: (Iuvec9|null);

    /** umat9 s7 */
    s7?: (Iuvec9|null);

    /** umat9 s8 */
    s8?: (Iuvec9|null);
}

/** Represents an umat9. */
export class umat9 implements Iumat9 {

    /**
     * Constructs a new umat9.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat9);

    /** umat9 s0. */
    public s0?: (Iuvec9|null);

    /** umat9 s1. */
    public s1?: (Iuvec9|null);

    /** umat9 s2. */
    public s2?: (Iuvec9|null);

    /** umat9 s3. */
    public s3?: (Iuvec9|null);

    /** umat9 s4. */
    public s4?: (Iuvec9|null);

    /** umat9 s5. */
    public s5?: (Iuvec9|null);

    /** umat9 s6. */
    public s6?: (Iuvec9|null);

    /** umat9 s7. */
    public s7?: (Iuvec9|null);

    /** umat9 s8. */
    public s8?: (Iuvec9|null);

    /**
     * Encodes the specified umat9 message. Does not implicitly {@link umat9.verify|verify} messages.
     * @param message umat9 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat9, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat9 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat9
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat9;
}

/** Properties of a mat10. */
export interface Imat10 {

    /** mat10 s0 */
    s0?: (Ivec10|null);

    /** mat10 s1 */
    s1?: (Ivec10|null);

    /** mat10 s2 */
    s2?: (Ivec10|null);

    /** mat10 s3 */
    s3?: (Ivec10|null);

    /** mat10 s4 */
    s4?: (Ivec10|null);

    /** mat10 s5 */
    s5?: (Ivec10|null);

    /** mat10 s6 */
    s6?: (Ivec10|null);

    /** mat10 s7 */
    s7?: (Ivec10|null);

    /** mat10 s8 */
    s8?: (Ivec10|null);

    /** mat10 s9 */
    s9?: (Ivec10|null);
}

/** Represents a mat10. */
export class mat10 implements Imat10 {

    /**
     * Constructs a new mat10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat10);

    /** mat10 s0. */
    public s0?: (Ivec10|null);

    /** mat10 s1. */
    public s1?: (Ivec10|null);

    /** mat10 s2. */
    public s2?: (Ivec10|null);

    /** mat10 s3. */
    public s3?: (Ivec10|null);

    /** mat10 s4. */
    public s4?: (Ivec10|null);

    /** mat10 s5. */
    public s5?: (Ivec10|null);

    /** mat10 s6. */
    public s6?: (Ivec10|null);

    /** mat10 s7. */
    public s7?: (Ivec10|null);

    /** mat10 s8. */
    public s8?: (Ivec10|null);

    /** mat10 s9. */
    public s9?: (Ivec10|null);

    /**
     * Encodes the specified mat10 message. Does not implicitly {@link mat10.verify|verify} messages.
     * @param message mat10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat10;
}

/** Properties of a fmat10. */
export interface Ifmat10 {

    /** fmat10 s0 */
    s0?: (Ifvec10|null);

    /** fmat10 s1 */
    s1?: (Ifvec10|null);

    /** fmat10 s2 */
    s2?: (Ifvec10|null);

    /** fmat10 s3 */
    s3?: (Ifvec10|null);

    /** fmat10 s4 */
    s4?: (Ifvec10|null);

    /** fmat10 s5 */
    s5?: (Ifvec10|null);

    /** fmat10 s6 */
    s6?: (Ifvec10|null);

    /** fmat10 s7 */
    s7?: (Ifvec10|null);

    /** fmat10 s8 */
    s8?: (Ifvec10|null);

    /** fmat10 s9 */
    s9?: (Ifvec10|null);
}

/** Represents a fmat10. */
export class fmat10 implements Ifmat10 {

    /**
     * Constructs a new fmat10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat10);

    /** fmat10 s0. */
    public s0?: (Ifvec10|null);

    /** fmat10 s1. */
    public s1?: (Ifvec10|null);

    /** fmat10 s2. */
    public s2?: (Ifvec10|null);

    /** fmat10 s3. */
    public s3?: (Ifvec10|null);

    /** fmat10 s4. */
    public s4?: (Ifvec10|null);

    /** fmat10 s5. */
    public s5?: (Ifvec10|null);

    /** fmat10 s6. */
    public s6?: (Ifvec10|null);

    /** fmat10 s7. */
    public s7?: (Ifvec10|null);

    /** fmat10 s8. */
    public s8?: (Ifvec10|null);

    /** fmat10 s9. */
    public s9?: (Ifvec10|null);

    /**
     * Encodes the specified fmat10 message. Does not implicitly {@link fmat10.verify|verify} messages.
     * @param message fmat10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat10;
}

/** Properties of an imat10. */
export interface Iimat10 {

    /** imat10 s0 */
    s0?: (Iivec10|null);

    /** imat10 s1 */
    s1?: (Iivec10|null);

    /** imat10 s2 */
    s2?: (Iivec10|null);

    /** imat10 s3 */
    s3?: (Iivec10|null);

    /** imat10 s4 */
    s4?: (Iivec10|null);

    /** imat10 s5 */
    s5?: (Iivec10|null);

    /** imat10 s6 */
    s6?: (Iivec10|null);

    /** imat10 s7 */
    s7?: (Iivec10|null);

    /** imat10 s8 */
    s8?: (Iivec10|null);

    /** imat10 s9 */
    s9?: (Iivec10|null);
}

/** Represents an imat10. */
export class imat10 implements Iimat10 {

    /**
     * Constructs a new imat10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat10);

    /** imat10 s0. */
    public s0?: (Iivec10|null);

    /** imat10 s1. */
    public s1?: (Iivec10|null);

    /** imat10 s2. */
    public s2?: (Iivec10|null);

    /** imat10 s3. */
    public s3?: (Iivec10|null);

    /** imat10 s4. */
    public s4?: (Iivec10|null);

    /** imat10 s5. */
    public s5?: (Iivec10|null);

    /** imat10 s6. */
    public s6?: (Iivec10|null);

    /** imat10 s7. */
    public s7?: (Iivec10|null);

    /** imat10 s8. */
    public s8?: (Iivec10|null);

    /** imat10 s9. */
    public s9?: (Iivec10|null);

    /**
     * Encodes the specified imat10 message. Does not implicitly {@link imat10.verify|verify} messages.
     * @param message imat10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat10;
}

/** Properties of an umat10. */
export interface Iumat10 {

    /** umat10 s0 */
    s0?: (Iuvec10|null);

    /** umat10 s1 */
    s1?: (Iuvec10|null);

    /** umat10 s2 */
    s2?: (Iuvec10|null);

    /** umat10 s3 */
    s3?: (Iuvec10|null);

    /** umat10 s4 */
    s4?: (Iuvec10|null);

    /** umat10 s5 */
    s5?: (Iuvec10|null);

    /** umat10 s6 */
    s6?: (Iuvec10|null);

    /** umat10 s7 */
    s7?: (Iuvec10|null);

    /** umat10 s8 */
    s8?: (Iuvec10|null);

    /** umat10 s9 */
    s9?: (Iuvec10|null);
}

/** Represents an umat10. */
export class umat10 implements Iumat10 {

    /**
     * Constructs a new umat10.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat10);

    /** umat10 s0. */
    public s0?: (Iuvec10|null);

    /** umat10 s1. */
    public s1?: (Iuvec10|null);

    /** umat10 s2. */
    public s2?: (Iuvec10|null);

    /** umat10 s3. */
    public s3?: (Iuvec10|null);

    /** umat10 s4. */
    public s4?: (Iuvec10|null);

    /** umat10 s5. */
    public s5?: (Iuvec10|null);

    /** umat10 s6. */
    public s6?: (Iuvec10|null);

    /** umat10 s7. */
    public s7?: (Iuvec10|null);

    /** umat10 s8. */
    public s8?: (Iuvec10|null);

    /** umat10 s9. */
    public s9?: (Iuvec10|null);

    /**
     * Encodes the specified umat10 message. Does not implicitly {@link umat10.verify|verify} messages.
     * @param message umat10 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat10, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat10 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat10
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat10;
}

/** Properties of a mat11. */
export interface Imat11 {

    /** mat11 s0 */
    s0?: (Ivec11|null);

    /** mat11 s1 */
    s1?: (Ivec11|null);

    /** mat11 s2 */
    s2?: (Ivec11|null);

    /** mat11 s3 */
    s3?: (Ivec11|null);

    /** mat11 s4 */
    s4?: (Ivec11|null);

    /** mat11 s5 */
    s5?: (Ivec11|null);

    /** mat11 s6 */
    s6?: (Ivec11|null);

    /** mat11 s7 */
    s7?: (Ivec11|null);

    /** mat11 s8 */
    s8?: (Ivec11|null);

    /** mat11 s9 */
    s9?: (Ivec11|null);

    /** mat11 sa */
    sa?: (Ivec11|null);
}

/** Represents a mat11. */
export class mat11 implements Imat11 {

    /**
     * Constructs a new mat11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat11);

    /** mat11 s0. */
    public s0?: (Ivec11|null);

    /** mat11 s1. */
    public s1?: (Ivec11|null);

    /** mat11 s2. */
    public s2?: (Ivec11|null);

    /** mat11 s3. */
    public s3?: (Ivec11|null);

    /** mat11 s4. */
    public s4?: (Ivec11|null);

    /** mat11 s5. */
    public s5?: (Ivec11|null);

    /** mat11 s6. */
    public s6?: (Ivec11|null);

    /** mat11 s7. */
    public s7?: (Ivec11|null);

    /** mat11 s8. */
    public s8?: (Ivec11|null);

    /** mat11 s9. */
    public s9?: (Ivec11|null);

    /** mat11 sa. */
    public sa?: (Ivec11|null);

    /**
     * Encodes the specified mat11 message. Does not implicitly {@link mat11.verify|verify} messages.
     * @param message mat11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat11;
}

/** Properties of a fmat11. */
export interface Ifmat11 {

    /** fmat11 s0 */
    s0?: (Ifvec11|null);

    /** fmat11 s1 */
    s1?: (Ifvec11|null);

    /** fmat11 s2 */
    s2?: (Ifvec11|null);

    /** fmat11 s3 */
    s3?: (Ifvec11|null);

    /** fmat11 s4 */
    s4?: (Ifvec11|null);

    /** fmat11 s5 */
    s5?: (Ifvec11|null);

    /** fmat11 s6 */
    s6?: (Ifvec11|null);

    /** fmat11 s7 */
    s7?: (Ifvec11|null);

    /** fmat11 s8 */
    s8?: (Ifvec11|null);

    /** fmat11 s9 */
    s9?: (Ifvec11|null);

    /** fmat11 sa */
    sa?: (Ifvec11|null);
}

/** Represents a fmat11. */
export class fmat11 implements Ifmat11 {

    /**
     * Constructs a new fmat11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat11);

    /** fmat11 s0. */
    public s0?: (Ifvec11|null);

    /** fmat11 s1. */
    public s1?: (Ifvec11|null);

    /** fmat11 s2. */
    public s2?: (Ifvec11|null);

    /** fmat11 s3. */
    public s3?: (Ifvec11|null);

    /** fmat11 s4. */
    public s4?: (Ifvec11|null);

    /** fmat11 s5. */
    public s5?: (Ifvec11|null);

    /** fmat11 s6. */
    public s6?: (Ifvec11|null);

    /** fmat11 s7. */
    public s7?: (Ifvec11|null);

    /** fmat11 s8. */
    public s8?: (Ifvec11|null);

    /** fmat11 s9. */
    public s9?: (Ifvec11|null);

    /** fmat11 sa. */
    public sa?: (Ifvec11|null);

    /**
     * Encodes the specified fmat11 message. Does not implicitly {@link fmat11.verify|verify} messages.
     * @param message fmat11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat11;
}

/** Properties of an imat11. */
export interface Iimat11 {

    /** imat11 s0 */
    s0?: (Iivec11|null);

    /** imat11 s1 */
    s1?: (Iivec11|null);

    /** imat11 s2 */
    s2?: (Iivec11|null);

    /** imat11 s3 */
    s3?: (Iivec11|null);

    /** imat11 s4 */
    s4?: (Iivec11|null);

    /** imat11 s5 */
    s5?: (Iivec11|null);

    /** imat11 s6 */
    s6?: (Iivec11|null);

    /** imat11 s7 */
    s7?: (Iivec11|null);

    /** imat11 s8 */
    s8?: (Iivec11|null);

    /** imat11 s9 */
    s9?: (Iivec11|null);

    /** imat11 sa */
    sa?: (Iivec11|null);
}

/** Represents an imat11. */
export class imat11 implements Iimat11 {

    /**
     * Constructs a new imat11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat11);

    /** imat11 s0. */
    public s0?: (Iivec11|null);

    /** imat11 s1. */
    public s1?: (Iivec11|null);

    /** imat11 s2. */
    public s2?: (Iivec11|null);

    /** imat11 s3. */
    public s3?: (Iivec11|null);

    /** imat11 s4. */
    public s4?: (Iivec11|null);

    /** imat11 s5. */
    public s5?: (Iivec11|null);

    /** imat11 s6. */
    public s6?: (Iivec11|null);

    /** imat11 s7. */
    public s7?: (Iivec11|null);

    /** imat11 s8. */
    public s8?: (Iivec11|null);

    /** imat11 s9. */
    public s9?: (Iivec11|null);

    /** imat11 sa. */
    public sa?: (Iivec11|null);

    /**
     * Encodes the specified imat11 message. Does not implicitly {@link imat11.verify|verify} messages.
     * @param message imat11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat11;
}

/** Properties of an umat11. */
export interface Iumat11 {

    /** umat11 s0 */
    s0?: (Iuvec11|null);

    /** umat11 s1 */
    s1?: (Iuvec11|null);

    /** umat11 s2 */
    s2?: (Iuvec11|null);

    /** umat11 s3 */
    s3?: (Iuvec11|null);

    /** umat11 s4 */
    s4?: (Iuvec11|null);

    /** umat11 s5 */
    s5?: (Iuvec11|null);

    /** umat11 s6 */
    s6?: (Iuvec11|null);

    /** umat11 s7 */
    s7?: (Iuvec11|null);

    /** umat11 s8 */
    s8?: (Iuvec11|null);

    /** umat11 s9 */
    s9?: (Iuvec11|null);

    /** umat11 sa */
    sa?: (Iuvec11|null);
}

/** Represents an umat11. */
export class umat11 implements Iumat11 {

    /**
     * Constructs a new umat11.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat11);

    /** umat11 s0. */
    public s0?: (Iuvec11|null);

    /** umat11 s1. */
    public s1?: (Iuvec11|null);

    /** umat11 s2. */
    public s2?: (Iuvec11|null);

    /** umat11 s3. */
    public s3?: (Iuvec11|null);

    /** umat11 s4. */
    public s4?: (Iuvec11|null);

    /** umat11 s5. */
    public s5?: (Iuvec11|null);

    /** umat11 s6. */
    public s6?: (Iuvec11|null);

    /** umat11 s7. */
    public s7?: (Iuvec11|null);

    /** umat11 s8. */
    public s8?: (Iuvec11|null);

    /** umat11 s9. */
    public s9?: (Iuvec11|null);

    /** umat11 sa. */
    public sa?: (Iuvec11|null);

    /**
     * Encodes the specified umat11 message. Does not implicitly {@link umat11.verify|verify} messages.
     * @param message umat11 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat11, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat11 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat11
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat11;
}

/** Properties of a mat12. */
export interface Imat12 {

    /** mat12 s0 */
    s0?: (Ivec12|null);

    /** mat12 s1 */
    s1?: (Ivec12|null);

    /** mat12 s2 */
    s2?: (Ivec12|null);

    /** mat12 s3 */
    s3?: (Ivec12|null);

    /** mat12 s4 */
    s4?: (Ivec12|null);

    /** mat12 s5 */
    s5?: (Ivec12|null);

    /** mat12 s6 */
    s6?: (Ivec12|null);

    /** mat12 s7 */
    s7?: (Ivec12|null);

    /** mat12 s8 */
    s8?: (Ivec12|null);

    /** mat12 s9 */
    s9?: (Ivec12|null);

    /** mat12 sa */
    sa?: (Ivec12|null);

    /** mat12 sb */
    sb?: (Ivec12|null);
}

/** Represents a mat12. */
export class mat12 implements Imat12 {

    /**
     * Constructs a new mat12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat12);

    /** mat12 s0. */
    public s0?: (Ivec12|null);

    /** mat12 s1. */
    public s1?: (Ivec12|null);

    /** mat12 s2. */
    public s2?: (Ivec12|null);

    /** mat12 s3. */
    public s3?: (Ivec12|null);

    /** mat12 s4. */
    public s4?: (Ivec12|null);

    /** mat12 s5. */
    public s5?: (Ivec12|null);

    /** mat12 s6. */
    public s6?: (Ivec12|null);

    /** mat12 s7. */
    public s7?: (Ivec12|null);

    /** mat12 s8. */
    public s8?: (Ivec12|null);

    /** mat12 s9. */
    public s9?: (Ivec12|null);

    /** mat12 sa. */
    public sa?: (Ivec12|null);

    /** mat12 sb. */
    public sb?: (Ivec12|null);

    /**
     * Encodes the specified mat12 message. Does not implicitly {@link mat12.verify|verify} messages.
     * @param message mat12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat12;
}

/** Properties of a fmat12. */
export interface Ifmat12 {

    /** fmat12 s0 */
    s0?: (Ifvec12|null);

    /** fmat12 s1 */
    s1?: (Ifvec12|null);

    /** fmat12 s2 */
    s2?: (Ifvec12|null);

    /** fmat12 s3 */
    s3?: (Ifvec12|null);

    /** fmat12 s4 */
    s4?: (Ifvec12|null);

    /** fmat12 s5 */
    s5?: (Ifvec12|null);

    /** fmat12 s6 */
    s6?: (Ifvec12|null);

    /** fmat12 s7 */
    s7?: (Ifvec12|null);

    /** fmat12 s8 */
    s8?: (Ifvec12|null);

    /** fmat12 s9 */
    s9?: (Ifvec12|null);

    /** fmat12 sa */
    sa?: (Ifvec12|null);

    /** fmat12 sb */
    sb?: (Ifvec12|null);
}

/** Represents a fmat12. */
export class fmat12 implements Ifmat12 {

    /**
     * Constructs a new fmat12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat12);

    /** fmat12 s0. */
    public s0?: (Ifvec12|null);

    /** fmat12 s1. */
    public s1?: (Ifvec12|null);

    /** fmat12 s2. */
    public s2?: (Ifvec12|null);

    /** fmat12 s3. */
    public s3?: (Ifvec12|null);

    /** fmat12 s4. */
    public s4?: (Ifvec12|null);

    /** fmat12 s5. */
    public s5?: (Ifvec12|null);

    /** fmat12 s6. */
    public s6?: (Ifvec12|null);

    /** fmat12 s7. */
    public s7?: (Ifvec12|null);

    /** fmat12 s8. */
    public s8?: (Ifvec12|null);

    /** fmat12 s9. */
    public s9?: (Ifvec12|null);

    /** fmat12 sa. */
    public sa?: (Ifvec12|null);

    /** fmat12 sb. */
    public sb?: (Ifvec12|null);

    /**
     * Encodes the specified fmat12 message. Does not implicitly {@link fmat12.verify|verify} messages.
     * @param message fmat12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat12;
}

/** Properties of an imat12. */
export interface Iimat12 {

    /** imat12 s0 */
    s0?: (Iivec12|null);

    /** imat12 s1 */
    s1?: (Iivec12|null);

    /** imat12 s2 */
    s2?: (Iivec12|null);

    /** imat12 s3 */
    s3?: (Iivec12|null);

    /** imat12 s4 */
    s4?: (Iivec12|null);

    /** imat12 s5 */
    s5?: (Iivec12|null);

    /** imat12 s6 */
    s6?: (Iivec12|null);

    /** imat12 s7 */
    s7?: (Iivec12|null);

    /** imat12 s8 */
    s8?: (Iivec12|null);

    /** imat12 s9 */
    s9?: (Iivec12|null);

    /** imat12 sa */
    sa?: (Iivec12|null);

    /** imat12 sb */
    sb?: (Iivec12|null);
}

/** Represents an imat12. */
export class imat12 implements Iimat12 {

    /**
     * Constructs a new imat12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat12);

    /** imat12 s0. */
    public s0?: (Iivec12|null);

    /** imat12 s1. */
    public s1?: (Iivec12|null);

    /** imat12 s2. */
    public s2?: (Iivec12|null);

    /** imat12 s3. */
    public s3?: (Iivec12|null);

    /** imat12 s4. */
    public s4?: (Iivec12|null);

    /** imat12 s5. */
    public s5?: (Iivec12|null);

    /** imat12 s6. */
    public s6?: (Iivec12|null);

    /** imat12 s7. */
    public s7?: (Iivec12|null);

    /** imat12 s8. */
    public s8?: (Iivec12|null);

    /** imat12 s9. */
    public s9?: (Iivec12|null);

    /** imat12 sa. */
    public sa?: (Iivec12|null);

    /** imat12 sb. */
    public sb?: (Iivec12|null);

    /**
     * Encodes the specified imat12 message. Does not implicitly {@link imat12.verify|verify} messages.
     * @param message imat12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat12;
}

/** Properties of an umat12. */
export interface Iumat12 {

    /** umat12 s0 */
    s0?: (Iuvec12|null);

    /** umat12 s1 */
    s1?: (Iuvec12|null);

    /** umat12 s2 */
    s2?: (Iuvec12|null);

    /** umat12 s3 */
    s3?: (Iuvec12|null);

    /** umat12 s4 */
    s4?: (Iuvec12|null);

    /** umat12 s5 */
    s5?: (Iuvec12|null);

    /** umat12 s6 */
    s6?: (Iuvec12|null);

    /** umat12 s7 */
    s7?: (Iuvec12|null);

    /** umat12 s8 */
    s8?: (Iuvec12|null);

    /** umat12 s9 */
    s9?: (Iuvec12|null);

    /** umat12 sa */
    sa?: (Iuvec12|null);

    /** umat12 sb */
    sb?: (Iuvec12|null);
}

/** Represents an umat12. */
export class umat12 implements Iumat12 {

    /**
     * Constructs a new umat12.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat12);

    /** umat12 s0. */
    public s0?: (Iuvec12|null);

    /** umat12 s1. */
    public s1?: (Iuvec12|null);

    /** umat12 s2. */
    public s2?: (Iuvec12|null);

    /** umat12 s3. */
    public s3?: (Iuvec12|null);

    /** umat12 s4. */
    public s4?: (Iuvec12|null);

    /** umat12 s5. */
    public s5?: (Iuvec12|null);

    /** umat12 s6. */
    public s6?: (Iuvec12|null);

    /** umat12 s7. */
    public s7?: (Iuvec12|null);

    /** umat12 s8. */
    public s8?: (Iuvec12|null);

    /** umat12 s9. */
    public s9?: (Iuvec12|null);

    /** umat12 sa. */
    public sa?: (Iuvec12|null);

    /** umat12 sb. */
    public sb?: (Iuvec12|null);

    /**
     * Encodes the specified umat12 message. Does not implicitly {@link umat12.verify|verify} messages.
     * @param message umat12 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat12, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat12 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat12
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat12;
}

/** Properties of a mat13. */
export interface Imat13 {

    /** mat13 s0 */
    s0?: (Ivec13|null);

    /** mat13 s1 */
    s1?: (Ivec13|null);

    /** mat13 s2 */
    s2?: (Ivec13|null);

    /** mat13 s3 */
    s3?: (Ivec13|null);

    /** mat13 s4 */
    s4?: (Ivec13|null);

    /** mat13 s5 */
    s5?: (Ivec13|null);

    /** mat13 s6 */
    s6?: (Ivec13|null);

    /** mat13 s7 */
    s7?: (Ivec13|null);

    /** mat13 s8 */
    s8?: (Ivec13|null);

    /** mat13 s9 */
    s9?: (Ivec13|null);

    /** mat13 sa */
    sa?: (Ivec13|null);

    /** mat13 sb */
    sb?: (Ivec13|null);

    /** mat13 sc */
    sc?: (Ivec13|null);
}

/** Represents a mat13. */
export class mat13 implements Imat13 {

    /**
     * Constructs a new mat13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat13);

    /** mat13 s0. */
    public s0?: (Ivec13|null);

    /** mat13 s1. */
    public s1?: (Ivec13|null);

    /** mat13 s2. */
    public s2?: (Ivec13|null);

    /** mat13 s3. */
    public s3?: (Ivec13|null);

    /** mat13 s4. */
    public s4?: (Ivec13|null);

    /** mat13 s5. */
    public s5?: (Ivec13|null);

    /** mat13 s6. */
    public s6?: (Ivec13|null);

    /** mat13 s7. */
    public s7?: (Ivec13|null);

    /** mat13 s8. */
    public s8?: (Ivec13|null);

    /** mat13 s9. */
    public s9?: (Ivec13|null);

    /** mat13 sa. */
    public sa?: (Ivec13|null);

    /** mat13 sb. */
    public sb?: (Ivec13|null);

    /** mat13 sc. */
    public sc?: (Ivec13|null);

    /**
     * Encodes the specified mat13 message. Does not implicitly {@link mat13.verify|verify} messages.
     * @param message mat13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat13;
}

/** Properties of a fmat13. */
export interface Ifmat13 {

    /** fmat13 s0 */
    s0?: (Ifvec13|null);

    /** fmat13 s1 */
    s1?: (Ifvec13|null);

    /** fmat13 s2 */
    s2?: (Ifvec13|null);

    /** fmat13 s3 */
    s3?: (Ifvec13|null);

    /** fmat13 s4 */
    s4?: (Ifvec13|null);

    /** fmat13 s5 */
    s5?: (Ifvec13|null);

    /** fmat13 s6 */
    s6?: (Ifvec13|null);

    /** fmat13 s7 */
    s7?: (Ifvec13|null);

    /** fmat13 s8 */
    s8?: (Ifvec13|null);

    /** fmat13 s9 */
    s9?: (Ifvec13|null);

    /** fmat13 sa */
    sa?: (Ifvec13|null);

    /** fmat13 sb */
    sb?: (Ifvec13|null);

    /** fmat13 sc */
    sc?: (Ifvec13|null);
}

/** Represents a fmat13. */
export class fmat13 implements Ifmat13 {

    /**
     * Constructs a new fmat13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat13);

    /** fmat13 s0. */
    public s0?: (Ifvec13|null);

    /** fmat13 s1. */
    public s1?: (Ifvec13|null);

    /** fmat13 s2. */
    public s2?: (Ifvec13|null);

    /** fmat13 s3. */
    public s3?: (Ifvec13|null);

    /** fmat13 s4. */
    public s4?: (Ifvec13|null);

    /** fmat13 s5. */
    public s5?: (Ifvec13|null);

    /** fmat13 s6. */
    public s6?: (Ifvec13|null);

    /** fmat13 s7. */
    public s7?: (Ifvec13|null);

    /** fmat13 s8. */
    public s8?: (Ifvec13|null);

    /** fmat13 s9. */
    public s9?: (Ifvec13|null);

    /** fmat13 sa. */
    public sa?: (Ifvec13|null);

    /** fmat13 sb. */
    public sb?: (Ifvec13|null);

    /** fmat13 sc. */
    public sc?: (Ifvec13|null);

    /**
     * Encodes the specified fmat13 message. Does not implicitly {@link fmat13.verify|verify} messages.
     * @param message fmat13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat13;
}

/** Properties of an imat13. */
export interface Iimat13 {

    /** imat13 s0 */
    s0?: (Iivec13|null);

    /** imat13 s1 */
    s1?: (Iivec13|null);

    /** imat13 s2 */
    s2?: (Iivec13|null);

    /** imat13 s3 */
    s3?: (Iivec13|null);

    /** imat13 s4 */
    s4?: (Iivec13|null);

    /** imat13 s5 */
    s5?: (Iivec13|null);

    /** imat13 s6 */
    s6?: (Iivec13|null);

    /** imat13 s7 */
    s7?: (Iivec13|null);

    /** imat13 s8 */
    s8?: (Iivec13|null);

    /** imat13 s9 */
    s9?: (Iivec13|null);

    /** imat13 sa */
    sa?: (Iivec13|null);

    /** imat13 sb */
    sb?: (Iivec13|null);

    /** imat13 sc */
    sc?: (Iivec13|null);
}

/** Represents an imat13. */
export class imat13 implements Iimat13 {

    /**
     * Constructs a new imat13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat13);

    /** imat13 s0. */
    public s0?: (Iivec13|null);

    /** imat13 s1. */
    public s1?: (Iivec13|null);

    /** imat13 s2. */
    public s2?: (Iivec13|null);

    /** imat13 s3. */
    public s3?: (Iivec13|null);

    /** imat13 s4. */
    public s4?: (Iivec13|null);

    /** imat13 s5. */
    public s5?: (Iivec13|null);

    /** imat13 s6. */
    public s6?: (Iivec13|null);

    /** imat13 s7. */
    public s7?: (Iivec13|null);

    /** imat13 s8. */
    public s8?: (Iivec13|null);

    /** imat13 s9. */
    public s9?: (Iivec13|null);

    /** imat13 sa. */
    public sa?: (Iivec13|null);

    /** imat13 sb. */
    public sb?: (Iivec13|null);

    /** imat13 sc. */
    public sc?: (Iivec13|null);

    /**
     * Encodes the specified imat13 message. Does not implicitly {@link imat13.verify|verify} messages.
     * @param message imat13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat13;
}

/** Properties of an umat13. */
export interface Iumat13 {

    /** umat13 s0 */
    s0?: (Iuvec13|null);

    /** umat13 s1 */
    s1?: (Iuvec13|null);

    /** umat13 s2 */
    s2?: (Iuvec13|null);

    /** umat13 s3 */
    s3?: (Iuvec13|null);

    /** umat13 s4 */
    s4?: (Iuvec13|null);

    /** umat13 s5 */
    s5?: (Iuvec13|null);

    /** umat13 s6 */
    s6?: (Iuvec13|null);

    /** umat13 s7 */
    s7?: (Iuvec13|null);

    /** umat13 s8 */
    s8?: (Iuvec13|null);

    /** umat13 s9 */
    s9?: (Iuvec13|null);

    /** umat13 sa */
    sa?: (Iuvec13|null);

    /** umat13 sb */
    sb?: (Iuvec13|null);

    /** umat13 sc */
    sc?: (Iuvec13|null);
}

/** Represents an umat13. */
export class umat13 implements Iumat13 {

    /**
     * Constructs a new umat13.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat13);

    /** umat13 s0. */
    public s0?: (Iuvec13|null);

    /** umat13 s1. */
    public s1?: (Iuvec13|null);

    /** umat13 s2. */
    public s2?: (Iuvec13|null);

    /** umat13 s3. */
    public s3?: (Iuvec13|null);

    /** umat13 s4. */
    public s4?: (Iuvec13|null);

    /** umat13 s5. */
    public s5?: (Iuvec13|null);

    /** umat13 s6. */
    public s6?: (Iuvec13|null);

    /** umat13 s7. */
    public s7?: (Iuvec13|null);

    /** umat13 s8. */
    public s8?: (Iuvec13|null);

    /** umat13 s9. */
    public s9?: (Iuvec13|null);

    /** umat13 sa. */
    public sa?: (Iuvec13|null);

    /** umat13 sb. */
    public sb?: (Iuvec13|null);

    /** umat13 sc. */
    public sc?: (Iuvec13|null);

    /**
     * Encodes the specified umat13 message. Does not implicitly {@link umat13.verify|verify} messages.
     * @param message umat13 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat13, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat13 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat13
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat13;
}

/** Properties of a mat14. */
export interface Imat14 {

    /** mat14 s0 */
    s0?: (Ivec14|null);

    /** mat14 s1 */
    s1?: (Ivec14|null);

    /** mat14 s2 */
    s2?: (Ivec14|null);

    /** mat14 s3 */
    s3?: (Ivec14|null);

    /** mat14 s4 */
    s4?: (Ivec14|null);

    /** mat14 s5 */
    s5?: (Ivec14|null);

    /** mat14 s6 */
    s6?: (Ivec14|null);

    /** mat14 s7 */
    s7?: (Ivec14|null);

    /** mat14 s8 */
    s8?: (Ivec14|null);

    /** mat14 s9 */
    s9?: (Ivec14|null);

    /** mat14 sa */
    sa?: (Ivec14|null);

    /** mat14 sb */
    sb?: (Ivec14|null);

    /** mat14 sc */
    sc?: (Ivec14|null);

    /** mat14 sd */
    sd?: (Ivec14|null);
}

/** Represents a mat14. */
export class mat14 implements Imat14 {

    /**
     * Constructs a new mat14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat14);

    /** mat14 s0. */
    public s0?: (Ivec14|null);

    /** mat14 s1. */
    public s1?: (Ivec14|null);

    /** mat14 s2. */
    public s2?: (Ivec14|null);

    /** mat14 s3. */
    public s3?: (Ivec14|null);

    /** mat14 s4. */
    public s4?: (Ivec14|null);

    /** mat14 s5. */
    public s5?: (Ivec14|null);

    /** mat14 s6. */
    public s6?: (Ivec14|null);

    /** mat14 s7. */
    public s7?: (Ivec14|null);

    /** mat14 s8. */
    public s8?: (Ivec14|null);

    /** mat14 s9. */
    public s9?: (Ivec14|null);

    /** mat14 sa. */
    public sa?: (Ivec14|null);

    /** mat14 sb. */
    public sb?: (Ivec14|null);

    /** mat14 sc. */
    public sc?: (Ivec14|null);

    /** mat14 sd. */
    public sd?: (Ivec14|null);

    /**
     * Encodes the specified mat14 message. Does not implicitly {@link mat14.verify|verify} messages.
     * @param message mat14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat14;
}

/** Properties of a fmat14. */
export interface Ifmat14 {

    /** fmat14 s0 */
    s0?: (Ifvec14|null);

    /** fmat14 s1 */
    s1?: (Ifvec14|null);

    /** fmat14 s2 */
    s2?: (Ifvec14|null);

    /** fmat14 s3 */
    s3?: (Ifvec14|null);

    /** fmat14 s4 */
    s4?: (Ifvec14|null);

    /** fmat14 s5 */
    s5?: (Ifvec14|null);

    /** fmat14 s6 */
    s6?: (Ifvec14|null);

    /** fmat14 s7 */
    s7?: (Ifvec14|null);

    /** fmat14 s8 */
    s8?: (Ifvec14|null);

    /** fmat14 s9 */
    s9?: (Ifvec14|null);

    /** fmat14 sa */
    sa?: (Ifvec14|null);

    /** fmat14 sb */
    sb?: (Ifvec14|null);

    /** fmat14 sc */
    sc?: (Ifvec14|null);

    /** fmat14 sd */
    sd?: (Ifvec14|null);
}

/** Represents a fmat14. */
export class fmat14 implements Ifmat14 {

    /**
     * Constructs a new fmat14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat14);

    /** fmat14 s0. */
    public s0?: (Ifvec14|null);

    /** fmat14 s1. */
    public s1?: (Ifvec14|null);

    /** fmat14 s2. */
    public s2?: (Ifvec14|null);

    /** fmat14 s3. */
    public s3?: (Ifvec14|null);

    /** fmat14 s4. */
    public s4?: (Ifvec14|null);

    /** fmat14 s5. */
    public s5?: (Ifvec14|null);

    /** fmat14 s6. */
    public s6?: (Ifvec14|null);

    /** fmat14 s7. */
    public s7?: (Ifvec14|null);

    /** fmat14 s8. */
    public s8?: (Ifvec14|null);

    /** fmat14 s9. */
    public s9?: (Ifvec14|null);

    /** fmat14 sa. */
    public sa?: (Ifvec14|null);

    /** fmat14 sb. */
    public sb?: (Ifvec14|null);

    /** fmat14 sc. */
    public sc?: (Ifvec14|null);

    /** fmat14 sd. */
    public sd?: (Ifvec14|null);

    /**
     * Encodes the specified fmat14 message. Does not implicitly {@link fmat14.verify|verify} messages.
     * @param message fmat14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat14;
}

/** Properties of an imat14. */
export interface Iimat14 {

    /** imat14 s0 */
    s0?: (Iivec14|null);

    /** imat14 s1 */
    s1?: (Iivec14|null);

    /** imat14 s2 */
    s2?: (Iivec14|null);

    /** imat14 s3 */
    s3?: (Iivec14|null);

    /** imat14 s4 */
    s4?: (Iivec14|null);

    /** imat14 s5 */
    s5?: (Iivec14|null);

    /** imat14 s6 */
    s6?: (Iivec14|null);

    /** imat14 s7 */
    s7?: (Iivec14|null);

    /** imat14 s8 */
    s8?: (Iivec14|null);

    /** imat14 s9 */
    s9?: (Iivec14|null);

    /** imat14 sa */
    sa?: (Iivec14|null);

    /** imat14 sb */
    sb?: (Iivec14|null);

    /** imat14 sc */
    sc?: (Iivec14|null);

    /** imat14 sd */
    sd?: (Iivec14|null);
}

/** Represents an imat14. */
export class imat14 implements Iimat14 {

    /**
     * Constructs a new imat14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat14);

    /** imat14 s0. */
    public s0?: (Iivec14|null);

    /** imat14 s1. */
    public s1?: (Iivec14|null);

    /** imat14 s2. */
    public s2?: (Iivec14|null);

    /** imat14 s3. */
    public s3?: (Iivec14|null);

    /** imat14 s4. */
    public s4?: (Iivec14|null);

    /** imat14 s5. */
    public s5?: (Iivec14|null);

    /** imat14 s6. */
    public s6?: (Iivec14|null);

    /** imat14 s7. */
    public s7?: (Iivec14|null);

    /** imat14 s8. */
    public s8?: (Iivec14|null);

    /** imat14 s9. */
    public s9?: (Iivec14|null);

    /** imat14 sa. */
    public sa?: (Iivec14|null);

    /** imat14 sb. */
    public sb?: (Iivec14|null);

    /** imat14 sc. */
    public sc?: (Iivec14|null);

    /** imat14 sd. */
    public sd?: (Iivec14|null);

    /**
     * Encodes the specified imat14 message. Does not implicitly {@link imat14.verify|verify} messages.
     * @param message imat14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat14;
}

/** Properties of an umat14. */
export interface Iumat14 {

    /** umat14 s0 */
    s0?: (Iuvec14|null);

    /** umat14 s1 */
    s1?: (Iuvec14|null);

    /** umat14 s2 */
    s2?: (Iuvec14|null);

    /** umat14 s3 */
    s3?: (Iuvec14|null);

    /** umat14 s4 */
    s4?: (Iuvec14|null);

    /** umat14 s5 */
    s5?: (Iuvec14|null);

    /** umat14 s6 */
    s6?: (Iuvec14|null);

    /** umat14 s7 */
    s7?: (Iuvec14|null);

    /** umat14 s8 */
    s8?: (Iuvec14|null);

    /** umat14 s9 */
    s9?: (Iuvec14|null);

    /** umat14 sa */
    sa?: (Iuvec14|null);

    /** umat14 sb */
    sb?: (Iuvec14|null);

    /** umat14 sc */
    sc?: (Iuvec14|null);

    /** umat14 sd */
    sd?: (Iuvec14|null);
}

/** Represents an umat14. */
export class umat14 implements Iumat14 {

    /**
     * Constructs a new umat14.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat14);

    /** umat14 s0. */
    public s0?: (Iuvec14|null);

    /** umat14 s1. */
    public s1?: (Iuvec14|null);

    /** umat14 s2. */
    public s2?: (Iuvec14|null);

    /** umat14 s3. */
    public s3?: (Iuvec14|null);

    /** umat14 s4. */
    public s4?: (Iuvec14|null);

    /** umat14 s5. */
    public s5?: (Iuvec14|null);

    /** umat14 s6. */
    public s6?: (Iuvec14|null);

    /** umat14 s7. */
    public s7?: (Iuvec14|null);

    /** umat14 s8. */
    public s8?: (Iuvec14|null);

    /** umat14 s9. */
    public s9?: (Iuvec14|null);

    /** umat14 sa. */
    public sa?: (Iuvec14|null);

    /** umat14 sb. */
    public sb?: (Iuvec14|null);

    /** umat14 sc. */
    public sc?: (Iuvec14|null);

    /** umat14 sd. */
    public sd?: (Iuvec14|null);

    /**
     * Encodes the specified umat14 message. Does not implicitly {@link umat14.verify|verify} messages.
     * @param message umat14 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat14, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat14 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat14
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat14;
}

/** Properties of a mat15. */
export interface Imat15 {

    /** mat15 s0 */
    s0?: (Ivec15|null);

    /** mat15 s1 */
    s1?: (Ivec15|null);

    /** mat15 s2 */
    s2?: (Ivec15|null);

    /** mat15 s3 */
    s3?: (Ivec15|null);

    /** mat15 s4 */
    s4?: (Ivec15|null);

    /** mat15 s5 */
    s5?: (Ivec15|null);

    /** mat15 s6 */
    s6?: (Ivec15|null);

    /** mat15 s7 */
    s7?: (Ivec15|null);

    /** mat15 s8 */
    s8?: (Ivec15|null);

    /** mat15 s9 */
    s9?: (Ivec15|null);

    /** mat15 sa */
    sa?: (Ivec15|null);

    /** mat15 sb */
    sb?: (Ivec15|null);

    /** mat15 sc */
    sc?: (Ivec15|null);

    /** mat15 sd */
    sd?: (Ivec15|null);

    /** mat15 se */
    se?: (Ivec15|null);
}

/** Represents a mat15. */
export class mat15 implements Imat15 {

    /**
     * Constructs a new mat15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat15);

    /** mat15 s0. */
    public s0?: (Ivec15|null);

    /** mat15 s1. */
    public s1?: (Ivec15|null);

    /** mat15 s2. */
    public s2?: (Ivec15|null);

    /** mat15 s3. */
    public s3?: (Ivec15|null);

    /** mat15 s4. */
    public s4?: (Ivec15|null);

    /** mat15 s5. */
    public s5?: (Ivec15|null);

    /** mat15 s6. */
    public s6?: (Ivec15|null);

    /** mat15 s7. */
    public s7?: (Ivec15|null);

    /** mat15 s8. */
    public s8?: (Ivec15|null);

    /** mat15 s9. */
    public s9?: (Ivec15|null);

    /** mat15 sa. */
    public sa?: (Ivec15|null);

    /** mat15 sb. */
    public sb?: (Ivec15|null);

    /** mat15 sc. */
    public sc?: (Ivec15|null);

    /** mat15 sd. */
    public sd?: (Ivec15|null);

    /** mat15 se. */
    public se?: (Ivec15|null);

    /**
     * Encodes the specified mat15 message. Does not implicitly {@link mat15.verify|verify} messages.
     * @param message mat15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat15;
}

/** Properties of a fmat15. */
export interface Ifmat15 {

    /** fmat15 s0 */
    s0?: (Ifvec15|null);

    /** fmat15 s1 */
    s1?: (Ifvec15|null);

    /** fmat15 s2 */
    s2?: (Ifvec15|null);

    /** fmat15 s3 */
    s3?: (Ifvec15|null);

    /** fmat15 s4 */
    s4?: (Ifvec15|null);

    /** fmat15 s5 */
    s5?: (Ifvec15|null);

    /** fmat15 s6 */
    s6?: (Ifvec15|null);

    /** fmat15 s7 */
    s7?: (Ifvec15|null);

    /** fmat15 s8 */
    s8?: (Ifvec15|null);

    /** fmat15 s9 */
    s9?: (Ifvec15|null);

    /** fmat15 sa */
    sa?: (Ifvec15|null);

    /** fmat15 sb */
    sb?: (Ifvec15|null);

    /** fmat15 sc */
    sc?: (Ifvec15|null);

    /** fmat15 sd */
    sd?: (Ifvec15|null);

    /** fmat15 se */
    se?: (Ifvec15|null);
}

/** Represents a fmat15. */
export class fmat15 implements Ifmat15 {

    /**
     * Constructs a new fmat15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat15);

    /** fmat15 s0. */
    public s0?: (Ifvec15|null);

    /** fmat15 s1. */
    public s1?: (Ifvec15|null);

    /** fmat15 s2. */
    public s2?: (Ifvec15|null);

    /** fmat15 s3. */
    public s3?: (Ifvec15|null);

    /** fmat15 s4. */
    public s4?: (Ifvec15|null);

    /** fmat15 s5. */
    public s5?: (Ifvec15|null);

    /** fmat15 s6. */
    public s6?: (Ifvec15|null);

    /** fmat15 s7. */
    public s7?: (Ifvec15|null);

    /** fmat15 s8. */
    public s8?: (Ifvec15|null);

    /** fmat15 s9. */
    public s9?: (Ifvec15|null);

    /** fmat15 sa. */
    public sa?: (Ifvec15|null);

    /** fmat15 sb. */
    public sb?: (Ifvec15|null);

    /** fmat15 sc. */
    public sc?: (Ifvec15|null);

    /** fmat15 sd. */
    public sd?: (Ifvec15|null);

    /** fmat15 se. */
    public se?: (Ifvec15|null);

    /**
     * Encodes the specified fmat15 message. Does not implicitly {@link fmat15.verify|verify} messages.
     * @param message fmat15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat15;
}

/** Properties of an imat15. */
export interface Iimat15 {

    /** imat15 s0 */
    s0?: (Iivec15|null);

    /** imat15 s1 */
    s1?: (Iivec15|null);

    /** imat15 s2 */
    s2?: (Iivec15|null);

    /** imat15 s3 */
    s3?: (Iivec15|null);

    /** imat15 s4 */
    s4?: (Iivec15|null);

    /** imat15 s5 */
    s5?: (Iivec15|null);

    /** imat15 s6 */
    s6?: (Iivec15|null);

    /** imat15 s7 */
    s7?: (Iivec15|null);

    /** imat15 s8 */
    s8?: (Iivec15|null);

    /** imat15 s9 */
    s9?: (Iivec15|null);

    /** imat15 sa */
    sa?: (Iivec15|null);

    /** imat15 sb */
    sb?: (Iivec15|null);

    /** imat15 sc */
    sc?: (Iivec15|null);

    /** imat15 sd */
    sd?: (Iivec15|null);

    /** imat15 se */
    se?: (Iivec15|null);
}

/** Represents an imat15. */
export class imat15 implements Iimat15 {

    /**
     * Constructs a new imat15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat15);

    /** imat15 s0. */
    public s0?: (Iivec15|null);

    /** imat15 s1. */
    public s1?: (Iivec15|null);

    /** imat15 s2. */
    public s2?: (Iivec15|null);

    /** imat15 s3. */
    public s3?: (Iivec15|null);

    /** imat15 s4. */
    public s4?: (Iivec15|null);

    /** imat15 s5. */
    public s5?: (Iivec15|null);

    /** imat15 s6. */
    public s6?: (Iivec15|null);

    /** imat15 s7. */
    public s7?: (Iivec15|null);

    /** imat15 s8. */
    public s8?: (Iivec15|null);

    /** imat15 s9. */
    public s9?: (Iivec15|null);

    /** imat15 sa. */
    public sa?: (Iivec15|null);

    /** imat15 sb. */
    public sb?: (Iivec15|null);

    /** imat15 sc. */
    public sc?: (Iivec15|null);

    /** imat15 sd. */
    public sd?: (Iivec15|null);

    /** imat15 se. */
    public se?: (Iivec15|null);

    /**
     * Encodes the specified imat15 message. Does not implicitly {@link imat15.verify|verify} messages.
     * @param message imat15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat15;
}

/** Properties of an umat15. */
export interface Iumat15 {

    /** umat15 s0 */
    s0?: (Iuvec15|null);

    /** umat15 s1 */
    s1?: (Iuvec15|null);

    /** umat15 s2 */
    s2?: (Iuvec15|null);

    /** umat15 s3 */
    s3?: (Iuvec15|null);

    /** umat15 s4 */
    s4?: (Iuvec15|null);

    /** umat15 s5 */
    s5?: (Iuvec15|null);

    /** umat15 s6 */
    s6?: (Iuvec15|null);

    /** umat15 s7 */
    s7?: (Iuvec15|null);

    /** umat15 s8 */
    s8?: (Iuvec15|null);

    /** umat15 s9 */
    s9?: (Iuvec15|null);

    /** umat15 sa */
    sa?: (Iuvec15|null);

    /** umat15 sb */
    sb?: (Iuvec15|null);

    /** umat15 sc */
    sc?: (Iuvec15|null);

    /** umat15 sd */
    sd?: (Iuvec15|null);

    /** umat15 se */
    se?: (Iuvec15|null);
}

/** Represents an umat15. */
export class umat15 implements Iumat15 {

    /**
     * Constructs a new umat15.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat15);

    /** umat15 s0. */
    public s0?: (Iuvec15|null);

    /** umat15 s1. */
    public s1?: (Iuvec15|null);

    /** umat15 s2. */
    public s2?: (Iuvec15|null);

    /** umat15 s3. */
    public s3?: (Iuvec15|null);

    /** umat15 s4. */
    public s4?: (Iuvec15|null);

    /** umat15 s5. */
    public s5?: (Iuvec15|null);

    /** umat15 s6. */
    public s6?: (Iuvec15|null);

    /** umat15 s7. */
    public s7?: (Iuvec15|null);

    /** umat15 s8. */
    public s8?: (Iuvec15|null);

    /** umat15 s9. */
    public s9?: (Iuvec15|null);

    /** umat15 sa. */
    public sa?: (Iuvec15|null);

    /** umat15 sb. */
    public sb?: (Iuvec15|null);

    /** umat15 sc. */
    public sc?: (Iuvec15|null);

    /** umat15 sd. */
    public sd?: (Iuvec15|null);

    /** umat15 se. */
    public se?: (Iuvec15|null);

    /**
     * Encodes the specified umat15 message. Does not implicitly {@link umat15.verify|verify} messages.
     * @param message umat15 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat15, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat15 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat15
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat15;
}

/** Properties of a mat16. */
export interface Imat16 {

    /** mat16 s0 */
    s0?: (Ivec16|null);

    /** mat16 s1 */
    s1?: (Ivec16|null);

    /** mat16 s2 */
    s2?: (Ivec16|null);

    /** mat16 s3 */
    s3?: (Ivec16|null);

    /** mat16 s4 */
    s4?: (Ivec16|null);

    /** mat16 s5 */
    s5?: (Ivec16|null);

    /** mat16 s6 */
    s6?: (Ivec16|null);

    /** mat16 s7 */
    s7?: (Ivec16|null);

    /** mat16 s8 */
    s8?: (Ivec16|null);

    /** mat16 s9 */
    s9?: (Ivec16|null);

    /** mat16 sa */
    sa?: (Ivec16|null);

    /** mat16 sb */
    sb?: (Ivec16|null);

    /** mat16 sc */
    sc?: (Ivec16|null);

    /** mat16 sd */
    sd?: (Ivec16|null);

    /** mat16 se */
    se?: (Ivec16|null);

    /** mat16 sf */
    sf?: (Ivec16|null);
}

/** Represents a mat16. */
export class mat16 implements Imat16 {

    /**
     * Constructs a new mat16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Imat16);

    /** mat16 s0. */
    public s0?: (Ivec16|null);

    /** mat16 s1. */
    public s1?: (Ivec16|null);

    /** mat16 s2. */
    public s2?: (Ivec16|null);

    /** mat16 s3. */
    public s3?: (Ivec16|null);

    /** mat16 s4. */
    public s4?: (Ivec16|null);

    /** mat16 s5. */
    public s5?: (Ivec16|null);

    /** mat16 s6. */
    public s6?: (Ivec16|null);

    /** mat16 s7. */
    public s7?: (Ivec16|null);

    /** mat16 s8. */
    public s8?: (Ivec16|null);

    /** mat16 s9. */
    public s9?: (Ivec16|null);

    /** mat16 sa. */
    public sa?: (Ivec16|null);

    /** mat16 sb. */
    public sb?: (Ivec16|null);

    /** mat16 sc. */
    public sc?: (Ivec16|null);

    /** mat16 sd. */
    public sd?: (Ivec16|null);

    /** mat16 se. */
    public se?: (Ivec16|null);

    /** mat16 sf. */
    public sf?: (Ivec16|null);

    /**
     * Encodes the specified mat16 message. Does not implicitly {@link mat16.verify|verify} messages.
     * @param message mat16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Imat16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a mat16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns mat16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): mat16;
}

/** Properties of a fmat16. */
export interface Ifmat16 {

    /** fmat16 s0 */
    s0?: (Ifvec16|null);

    /** fmat16 s1 */
    s1?: (Ifvec16|null);

    /** fmat16 s2 */
    s2?: (Ifvec16|null);

    /** fmat16 s3 */
    s3?: (Ifvec16|null);

    /** fmat16 s4 */
    s4?: (Ifvec16|null);

    /** fmat16 s5 */
    s5?: (Ifvec16|null);

    /** fmat16 s6 */
    s6?: (Ifvec16|null);

    /** fmat16 s7 */
    s7?: (Ifvec16|null);

    /** fmat16 s8 */
    s8?: (Ifvec16|null);

    /** fmat16 s9 */
    s9?: (Ifvec16|null);

    /** fmat16 sa */
    sa?: (Ifvec16|null);

    /** fmat16 sb */
    sb?: (Ifvec16|null);

    /** fmat16 sc */
    sc?: (Ifvec16|null);

    /** fmat16 sd */
    sd?: (Ifvec16|null);

    /** fmat16 se */
    se?: (Ifvec16|null);

    /** fmat16 sf */
    sf?: (Ifvec16|null);
}

/** Represents a fmat16. */
export class fmat16 implements Ifmat16 {

    /**
     * Constructs a new fmat16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Ifmat16);

    /** fmat16 s0. */
    public s0?: (Ifvec16|null);

    /** fmat16 s1. */
    public s1?: (Ifvec16|null);

    /** fmat16 s2. */
    public s2?: (Ifvec16|null);

    /** fmat16 s3. */
    public s3?: (Ifvec16|null);

    /** fmat16 s4. */
    public s4?: (Ifvec16|null);

    /** fmat16 s5. */
    public s5?: (Ifvec16|null);

    /** fmat16 s6. */
    public s6?: (Ifvec16|null);

    /** fmat16 s7. */
    public s7?: (Ifvec16|null);

    /** fmat16 s8. */
    public s8?: (Ifvec16|null);

    /** fmat16 s9. */
    public s9?: (Ifvec16|null);

    /** fmat16 sa. */
    public sa?: (Ifvec16|null);

    /** fmat16 sb. */
    public sb?: (Ifvec16|null);

    /** fmat16 sc. */
    public sc?: (Ifvec16|null);

    /** fmat16 sd. */
    public sd?: (Ifvec16|null);

    /** fmat16 se. */
    public se?: (Ifvec16|null);

    /** fmat16 sf. */
    public sf?: (Ifvec16|null);

    /**
     * Encodes the specified fmat16 message. Does not implicitly {@link fmat16.verify|verify} messages.
     * @param message fmat16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Ifmat16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a fmat16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns fmat16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): fmat16;
}

/** Properties of an imat16. */
export interface Iimat16 {

    /** imat16 s0 */
    s0?: (Iivec16|null);

    /** imat16 s1 */
    s1?: (Iivec16|null);

    /** imat16 s2 */
    s2?: (Iivec16|null);

    /** imat16 s3 */
    s3?: (Iivec16|null);

    /** imat16 s4 */
    s4?: (Iivec16|null);

    /** imat16 s5 */
    s5?: (Iivec16|null);

    /** imat16 s6 */
    s6?: (Iivec16|null);

    /** imat16 s7 */
    s7?: (Iivec16|null);

    /** imat16 s8 */
    s8?: (Iivec16|null);

    /** imat16 s9 */
    s9?: (Iivec16|null);

    /** imat16 sa */
    sa?: (Iivec16|null);

    /** imat16 sb */
    sb?: (Iivec16|null);

    /** imat16 sc */
    sc?: (Iivec16|null);

    /** imat16 sd */
    sd?: (Iivec16|null);

    /** imat16 se */
    se?: (Iivec16|null);

    /** imat16 sf */
    sf?: (Iivec16|null);
}

/** Represents an imat16. */
export class imat16 implements Iimat16 {

    /**
     * Constructs a new imat16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iimat16);

    /** imat16 s0. */
    public s0?: (Iivec16|null);

    /** imat16 s1. */
    public s1?: (Iivec16|null);

    /** imat16 s2. */
    public s2?: (Iivec16|null);

    /** imat16 s3. */
    public s3?: (Iivec16|null);

    /** imat16 s4. */
    public s4?: (Iivec16|null);

    /** imat16 s5. */
    public s5?: (Iivec16|null);

    /** imat16 s6. */
    public s6?: (Iivec16|null);

    /** imat16 s7. */
    public s7?: (Iivec16|null);

    /** imat16 s8. */
    public s8?: (Iivec16|null);

    /** imat16 s9. */
    public s9?: (Iivec16|null);

    /** imat16 sa. */
    public sa?: (Iivec16|null);

    /** imat16 sb. */
    public sb?: (Iivec16|null);

    /** imat16 sc. */
    public sc?: (Iivec16|null);

    /** imat16 sd. */
    public sd?: (Iivec16|null);

    /** imat16 se. */
    public se?: (Iivec16|null);

    /** imat16 sf. */
    public sf?: (Iivec16|null);

    /**
     * Encodes the specified imat16 message. Does not implicitly {@link imat16.verify|verify} messages.
     * @param message imat16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iimat16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an imat16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns imat16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): imat16;
}

/** Properties of an umat16. */
export interface Iumat16 {

    /** umat16 s0 */
    s0?: (Iuvec16|null);

    /** umat16 s1 */
    s1?: (Iuvec16|null);

    /** umat16 s2 */
    s2?: (Iuvec16|null);

    /** umat16 s3 */
    s3?: (Iuvec16|null);

    /** umat16 s4 */
    s4?: (Iuvec16|null);

    /** umat16 s5 */
    s5?: (Iuvec16|null);

    /** umat16 s6 */
    s6?: (Iuvec16|null);

    /** umat16 s7 */
    s7?: (Iuvec16|null);

    /** umat16 s8 */
    s8?: (Iuvec16|null);

    /** umat16 s9 */
    s9?: (Iuvec16|null);

    /** umat16 sa */
    sa?: (Iuvec16|null);

    /** umat16 sb */
    sb?: (Iuvec16|null);

    /** umat16 sc */
    sc?: (Iuvec16|null);

    /** umat16 sd */
    sd?: (Iuvec16|null);

    /** umat16 se */
    se?: (Iuvec16|null);

    /** umat16 sf */
    sf?: (Iuvec16|null);
}

/** Represents an umat16. */
export class umat16 implements Iumat16 {

    /**
     * Constructs a new umat16.
     * @param [properties] Properties to set
     */
    constructor(properties?: Iumat16);

    /** umat16 s0. */
    public s0?: (Iuvec16|null);

    /** umat16 s1. */
    public s1?: (Iuvec16|null);

    /** umat16 s2. */
    public s2?: (Iuvec16|null);

    /** umat16 s3. */
    public s3?: (Iuvec16|null);

    /** umat16 s4. */
    public s4?: (Iuvec16|null);

    /** umat16 s5. */
    public s5?: (Iuvec16|null);

    /** umat16 s6. */
    public s6?: (Iuvec16|null);

    /** umat16 s7. */
    public s7?: (Iuvec16|null);

    /** umat16 s8. */
    public s8?: (Iuvec16|null);

    /** umat16 s9. */
    public s9?: (Iuvec16|null);

    /** umat16 sa. */
    public sa?: (Iuvec16|null);

    /** umat16 sb. */
    public sb?: (Iuvec16|null);

    /** umat16 sc. */
    public sc?: (Iuvec16|null);

    /** umat16 sd. */
    public sd?: (Iuvec16|null);

    /** umat16 se. */
    public se?: (Iuvec16|null);

    /** umat16 sf. */
    public sf?: (Iuvec16|null);

    /**
     * Encodes the specified umat16 message. Does not implicitly {@link umat16.verify|verify} messages.
     * @param message umat16 message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: Iumat16, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes an umat16 message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns umat16
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): umat16;
}

/** Namespace google. */
export namespace google {

    /** Namespace protobuf. */
    namespace protobuf {

        /** Properties of a Timestamp. */
        interface ITimestamp {

            /** Timestamp seconds */
            seconds?: (number|Long|null);

            /** Timestamp nanos */
            nanos?: (number|null);
        }

        /** Represents a Timestamp. */
        class Timestamp implements ITimestamp {

            /**
             * Constructs a new Timestamp.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.ITimestamp);

            /** Timestamp seconds. */
            public seconds: (number|Long);

            /** Timestamp nanos. */
            public nanos: number;

            /**
             * Encodes the specified Timestamp message. Does not implicitly {@link google.protobuf.Timestamp.verify|verify} messages.
             * @param message Timestamp message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.ITimestamp, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Timestamp message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Timestamp
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.Timestamp;
        }

        /** Properties of a FileDescriptorSet. */
        interface IFileDescriptorSet {

            /** FileDescriptorSet file */
            file?: (google.protobuf.IFileDescriptorProto[]|null);
        }

        /** Represents a FileDescriptorSet. */
        class FileDescriptorSet implements IFileDescriptorSet {

            /**
             * Constructs a new FileDescriptorSet.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IFileDescriptorSet);

            /** FileDescriptorSet file. */
            public file: google.protobuf.IFileDescriptorProto[];

            /**
             * Encodes the specified FileDescriptorSet message. Does not implicitly {@link google.protobuf.FileDescriptorSet.verify|verify} messages.
             * @param message FileDescriptorSet message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IFileDescriptorSet, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FileDescriptorSet message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FileDescriptorSet
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.FileDescriptorSet;
        }

        /** Properties of a FileDescriptorProto. */
        interface IFileDescriptorProto {

            /** FileDescriptorProto name */
            name?: (string|null);

            /** FileDescriptorProto package */
            "package"?: (string|null);

            /** FileDescriptorProto dependency */
            dependency?: (string[]|null);

            /** FileDescriptorProto publicDependency */
            publicDependency?: (number[]|null);

            /** FileDescriptorProto weakDependency */
            weakDependency?: (number[]|null);

            /** FileDescriptorProto messageType */
            messageType?: (google.protobuf.IDescriptorProto[]|null);

            /** FileDescriptorProto enumType */
            enumType?: (google.protobuf.IEnumDescriptorProto[]|null);

            /** FileDescriptorProto service */
            service?: (google.protobuf.IServiceDescriptorProto[]|null);

            /** FileDescriptorProto extension */
            extension?: (google.protobuf.IFieldDescriptorProto[]|null);

            /** FileDescriptorProto options */
            options?: (google.protobuf.IFileOptions|null);

            /** FileDescriptorProto sourceCodeInfo */
            sourceCodeInfo?: (google.protobuf.ISourceCodeInfo|null);

            /** FileDescriptorProto syntax */
            syntax?: (string|null);
        }

        /** Represents a FileDescriptorProto. */
        class FileDescriptorProto implements IFileDescriptorProto {

            /**
             * Constructs a new FileDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IFileDescriptorProto);

            /** FileDescriptorProto name. */
            public name: string;

            /** FileDescriptorProto package. */
            public package: string;

            /** FileDescriptorProto dependency. */
            public dependency: string[];

            /** FileDescriptorProto publicDependency. */
            public publicDependency: number[];

            /** FileDescriptorProto weakDependency. */
            public weakDependency: number[];

            /** FileDescriptorProto messageType. */
            public messageType: google.protobuf.IDescriptorProto[];

            /** FileDescriptorProto enumType. */
            public enumType: google.protobuf.IEnumDescriptorProto[];

            /** FileDescriptorProto service. */
            public service: google.protobuf.IServiceDescriptorProto[];

            /** FileDescriptorProto extension. */
            public extension: google.protobuf.IFieldDescriptorProto[];

            /** FileDescriptorProto options. */
            public options?: (google.protobuf.IFileOptions|null);

            /** FileDescriptorProto sourceCodeInfo. */
            public sourceCodeInfo?: (google.protobuf.ISourceCodeInfo|null);

            /** FileDescriptorProto syntax. */
            public syntax: string;

            /**
             * Encodes the specified FileDescriptorProto message. Does not implicitly {@link google.protobuf.FileDescriptorProto.verify|verify} messages.
             * @param message FileDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IFileDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FileDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FileDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.FileDescriptorProto;
        }

        /** Properties of a DescriptorProto. */
        interface IDescriptorProto {

            /** DescriptorProto name */
            name?: (string|null);

            /** DescriptorProto field */
            field?: (google.protobuf.IFieldDescriptorProto[]|null);

            /** DescriptorProto extension */
            extension?: (google.protobuf.IFieldDescriptorProto[]|null);

            /** DescriptorProto nestedType */
            nestedType?: (google.protobuf.IDescriptorProto[]|null);

            /** DescriptorProto enumType */
            enumType?: (google.protobuf.IEnumDescriptorProto[]|null);

            /** DescriptorProto extensionRange */
            extensionRange?: (google.protobuf.DescriptorProto.IExtensionRange[]|null);

            /** DescriptorProto oneofDecl */
            oneofDecl?: (google.protobuf.IOneofDescriptorProto[]|null);

            /** DescriptorProto options */
            options?: (google.protobuf.IMessageOptions|null);

            /** DescriptorProto reservedRange */
            reservedRange?: (google.protobuf.DescriptorProto.IReservedRange[]|null);

            /** DescriptorProto reservedName */
            reservedName?: (string[]|null);
        }

        /** Represents a DescriptorProto. */
        class DescriptorProto implements IDescriptorProto {

            /**
             * Constructs a new DescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IDescriptorProto);

            /** DescriptorProto name. */
            public name: string;

            /** DescriptorProto field. */
            public field: google.protobuf.IFieldDescriptorProto[];

            /** DescriptorProto extension. */
            public extension: google.protobuf.IFieldDescriptorProto[];

            /** DescriptorProto nestedType. */
            public nestedType: google.protobuf.IDescriptorProto[];

            /** DescriptorProto enumType. */
            public enumType: google.protobuf.IEnumDescriptorProto[];

            /** DescriptorProto extensionRange. */
            public extensionRange: google.protobuf.DescriptorProto.IExtensionRange[];

            /** DescriptorProto oneofDecl. */
            public oneofDecl: google.protobuf.IOneofDescriptorProto[];

            /** DescriptorProto options. */
            public options?: (google.protobuf.IMessageOptions|null);

            /** DescriptorProto reservedRange. */
            public reservedRange: google.protobuf.DescriptorProto.IReservedRange[];

            /** DescriptorProto reservedName. */
            public reservedName: string[];

            /**
             * Encodes the specified DescriptorProto message. Does not implicitly {@link google.protobuf.DescriptorProto.verify|verify} messages.
             * @param message DescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.DescriptorProto;
        }

        namespace DescriptorProto {

            /** Properties of an ExtensionRange. */
            interface IExtensionRange {

                /** ExtensionRange start */
                start?: (number|null);

                /** ExtensionRange end */
                end?: (number|null);
            }

            /** Represents an ExtensionRange. */
            class ExtensionRange implements IExtensionRange {

                /**
                 * Constructs a new ExtensionRange.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: google.protobuf.DescriptorProto.IExtensionRange);

                /** ExtensionRange start. */
                public start: number;

                /** ExtensionRange end. */
                public end: number;

                /**
                 * Encodes the specified ExtensionRange message. Does not implicitly {@link google.protobuf.DescriptorProto.ExtensionRange.verify|verify} messages.
                 * @param message ExtensionRange message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: google.protobuf.DescriptorProto.IExtensionRange, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an ExtensionRange message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ExtensionRange
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.DescriptorProto.ExtensionRange;
            }

            /** Properties of a ReservedRange. */
            interface IReservedRange {

                /** ReservedRange start */
                start?: (number|null);

                /** ReservedRange end */
                end?: (number|null);
            }

            /** Represents a ReservedRange. */
            class ReservedRange implements IReservedRange {

                /**
                 * Constructs a new ReservedRange.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: google.protobuf.DescriptorProto.IReservedRange);

                /** ReservedRange start. */
                public start: number;

                /** ReservedRange end. */
                public end: number;

                /**
                 * Encodes the specified ReservedRange message. Does not implicitly {@link google.protobuf.DescriptorProto.ReservedRange.verify|verify} messages.
                 * @param message ReservedRange message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: google.protobuf.DescriptorProto.IReservedRange, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a ReservedRange message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns ReservedRange
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.DescriptorProto.ReservedRange;
            }
        }

        /** Properties of a FieldDescriptorProto. */
        interface IFieldDescriptorProto {

            /** FieldDescriptorProto name */
            name?: (string|null);

            /** FieldDescriptorProto number */
            number?: (number|null);

            /** FieldDescriptorProto label */
            label?: (google.protobuf.FieldDescriptorProto.Label|null);

            /** FieldDescriptorProto type */
            type?: (google.protobuf.FieldDescriptorProto.Type|null);

            /** FieldDescriptorProto typeName */
            typeName?: (string|null);

            /** FieldDescriptorProto extendee */
            extendee?: (string|null);

            /** FieldDescriptorProto defaultValue */
            defaultValue?: (string|null);

            /** FieldDescriptorProto oneofIndex */
            oneofIndex?: (number|null);

            /** FieldDescriptorProto jsonName */
            jsonName?: (string|null);

            /** FieldDescriptorProto options */
            options?: (google.protobuf.IFieldOptions|null);
        }

        /** Represents a FieldDescriptorProto. */
        class FieldDescriptorProto implements IFieldDescriptorProto {

            /**
             * Constructs a new FieldDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IFieldDescriptorProto);

            /** FieldDescriptorProto name. */
            public name: string;

            /** FieldDescriptorProto number. */
            public number: number;

            /** FieldDescriptorProto label. */
            public label: google.protobuf.FieldDescriptorProto.Label;

            /** FieldDescriptorProto type. */
            public type: google.protobuf.FieldDescriptorProto.Type;

            /** FieldDescriptorProto typeName. */
            public typeName: string;

            /** FieldDescriptorProto extendee. */
            public extendee: string;

            /** FieldDescriptorProto defaultValue. */
            public defaultValue: string;

            /** FieldDescriptorProto oneofIndex. */
            public oneofIndex: number;

            /** FieldDescriptorProto jsonName. */
            public jsonName: string;

            /** FieldDescriptorProto options. */
            public options?: (google.protobuf.IFieldOptions|null);

            /**
             * Encodes the specified FieldDescriptorProto message. Does not implicitly {@link google.protobuf.FieldDescriptorProto.verify|verify} messages.
             * @param message FieldDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IFieldDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FieldDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FieldDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.FieldDescriptorProto;
        }

        namespace FieldDescriptorProto {

            /** Type enum. */
            enum Type {
                TYPE_DOUBLE = 1,
                TYPE_FLOAT = 2,
                TYPE_INT64 = 3,
                TYPE_UINT64 = 4,
                TYPE_INT32 = 5,
                TYPE_FIXED64 = 6,
                TYPE_FIXED32 = 7,
                TYPE_BOOL = 8,
                TYPE_STRING = 9,
                TYPE_GROUP = 10,
                TYPE_MESSAGE = 11,
                TYPE_BYTES = 12,
                TYPE_UINT32 = 13,
                TYPE_ENUM = 14,
                TYPE_SFIXED32 = 15,
                TYPE_SFIXED64 = 16,
                TYPE_SINT32 = 17,
                TYPE_SINT64 = 18
            }

            /** Label enum. */
            enum Label {
                LABEL_OPTIONAL = 1,
                LABEL_REQUIRED = 2,
                LABEL_REPEATED = 3
            }
        }

        /** Properties of an OneofDescriptorProto. */
        interface IOneofDescriptorProto {

            /** OneofDescriptorProto name */
            name?: (string|null);

            /** OneofDescriptorProto options */
            options?: (google.protobuf.IOneofOptions|null);
        }

        /** Represents an OneofDescriptorProto. */
        class OneofDescriptorProto implements IOneofDescriptorProto {

            /**
             * Constructs a new OneofDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IOneofDescriptorProto);

            /** OneofDescriptorProto name. */
            public name: string;

            /** OneofDescriptorProto options. */
            public options?: (google.protobuf.IOneofOptions|null);

            /**
             * Encodes the specified OneofDescriptorProto message. Does not implicitly {@link google.protobuf.OneofDescriptorProto.verify|verify} messages.
             * @param message OneofDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IOneofDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an OneofDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns OneofDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.OneofDescriptorProto;
        }

        /** Properties of an EnumDescriptorProto. */
        interface IEnumDescriptorProto {

            /** EnumDescriptorProto name */
            name?: (string|null);

            /** EnumDescriptorProto value */
            value?: (google.protobuf.IEnumValueDescriptorProto[]|null);

            /** EnumDescriptorProto options */
            options?: (google.protobuf.IEnumOptions|null);
        }

        /** Represents an EnumDescriptorProto. */
        class EnumDescriptorProto implements IEnumDescriptorProto {

            /**
             * Constructs a new EnumDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IEnumDescriptorProto);

            /** EnumDescriptorProto name. */
            public name: string;

            /** EnumDescriptorProto value. */
            public value: google.protobuf.IEnumValueDescriptorProto[];

            /** EnumDescriptorProto options. */
            public options?: (google.protobuf.IEnumOptions|null);

            /**
             * Encodes the specified EnumDescriptorProto message. Does not implicitly {@link google.protobuf.EnumDescriptorProto.verify|verify} messages.
             * @param message EnumDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IEnumDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EnumDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EnumDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.EnumDescriptorProto;
        }

        /** Properties of an EnumValueDescriptorProto. */
        interface IEnumValueDescriptorProto {

            /** EnumValueDescriptorProto name */
            name?: (string|null);

            /** EnumValueDescriptorProto number */
            number?: (number|null);

            /** EnumValueDescriptorProto options */
            options?: (google.protobuf.IEnumValueOptions|null);
        }

        /** Represents an EnumValueDescriptorProto. */
        class EnumValueDescriptorProto implements IEnumValueDescriptorProto {

            /**
             * Constructs a new EnumValueDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IEnumValueDescriptorProto);

            /** EnumValueDescriptorProto name. */
            public name: string;

            /** EnumValueDescriptorProto number. */
            public number: number;

            /** EnumValueDescriptorProto options. */
            public options?: (google.protobuf.IEnumValueOptions|null);

            /**
             * Encodes the specified EnumValueDescriptorProto message. Does not implicitly {@link google.protobuf.EnumValueDescriptorProto.verify|verify} messages.
             * @param message EnumValueDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IEnumValueDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EnumValueDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EnumValueDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.EnumValueDescriptorProto;
        }

        /** Properties of a ServiceDescriptorProto. */
        interface IServiceDescriptorProto {

            /** ServiceDescriptorProto name */
            name?: (string|null);

            /** ServiceDescriptorProto method */
            method?: (google.protobuf.IMethodDescriptorProto[]|null);

            /** ServiceDescriptorProto options */
            options?: (google.protobuf.IServiceOptions|null);
        }

        /** Represents a ServiceDescriptorProto. */
        class ServiceDescriptorProto implements IServiceDescriptorProto {

            /**
             * Constructs a new ServiceDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IServiceDescriptorProto);

            /** ServiceDescriptorProto name. */
            public name: string;

            /** ServiceDescriptorProto method. */
            public method: google.protobuf.IMethodDescriptorProto[];

            /** ServiceDescriptorProto options. */
            public options?: (google.protobuf.IServiceOptions|null);

            /**
             * Encodes the specified ServiceDescriptorProto message. Does not implicitly {@link google.protobuf.ServiceDescriptorProto.verify|verify} messages.
             * @param message ServiceDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IServiceDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServiceDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServiceDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.ServiceDescriptorProto;
        }

        /** Properties of a MethodDescriptorProto. */
        interface IMethodDescriptorProto {

            /** MethodDescriptorProto name */
            name?: (string|null);

            /** MethodDescriptorProto inputType */
            inputType?: (string|null);

            /** MethodDescriptorProto outputType */
            outputType?: (string|null);

            /** MethodDescriptorProto options */
            options?: (google.protobuf.IMethodOptions|null);

            /** MethodDescriptorProto clientStreaming */
            clientStreaming?: (boolean|null);

            /** MethodDescriptorProto serverStreaming */
            serverStreaming?: (boolean|null);
        }

        /** Represents a MethodDescriptorProto. */
        class MethodDescriptorProto implements IMethodDescriptorProto {

            /**
             * Constructs a new MethodDescriptorProto.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IMethodDescriptorProto);

            /** MethodDescriptorProto name. */
            public name: string;

            /** MethodDescriptorProto inputType. */
            public inputType: string;

            /** MethodDescriptorProto outputType. */
            public outputType: string;

            /** MethodDescriptorProto options. */
            public options?: (google.protobuf.IMethodOptions|null);

            /** MethodDescriptorProto clientStreaming. */
            public clientStreaming: boolean;

            /** MethodDescriptorProto serverStreaming. */
            public serverStreaming: boolean;

            /**
             * Encodes the specified MethodDescriptorProto message. Does not implicitly {@link google.protobuf.MethodDescriptorProto.verify|verify} messages.
             * @param message MethodDescriptorProto message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IMethodDescriptorProto, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MethodDescriptorProto message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MethodDescriptorProto
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.MethodDescriptorProto;
        }

        /** Properties of a FileOptions. */
        interface IFileOptions {

            /** FileOptions javaPackage */
            javaPackage?: (string|null);

            /** FileOptions javaOuterClassname */
            javaOuterClassname?: (string|null);

            /** FileOptions javaMultipleFiles */
            javaMultipleFiles?: (boolean|null);

            /** FileOptions javaGenerateEqualsAndHash */
            javaGenerateEqualsAndHash?: (boolean|null);

            /** FileOptions javaStringCheckUtf8 */
            javaStringCheckUtf8?: (boolean|null);

            /** FileOptions optimizeFor */
            optimizeFor?: (google.protobuf.FileOptions.OptimizeMode|null);

            /** FileOptions goPackage */
            goPackage?: (string|null);

            /** FileOptions ccGenericServices */
            ccGenericServices?: (boolean|null);

            /** FileOptions javaGenericServices */
            javaGenericServices?: (boolean|null);

            /** FileOptions pyGenericServices */
            pyGenericServices?: (boolean|null);

            /** FileOptions deprecated */
            deprecated?: (boolean|null);

            /** FileOptions ccEnableArenas */
            ccEnableArenas?: (boolean|null);

            /** FileOptions objcClassPrefix */
            objcClassPrefix?: (string|null);

            /** FileOptions csharpNamespace */
            csharpNamespace?: (string|null);

            /** FileOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents a FileOptions. */
        class FileOptions implements IFileOptions {

            /**
             * Constructs a new FileOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IFileOptions);

            /** FileOptions javaPackage. */
            public javaPackage: string;

            /** FileOptions javaOuterClassname. */
            public javaOuterClassname: string;

            /** FileOptions javaMultipleFiles. */
            public javaMultipleFiles: boolean;

            /** FileOptions javaGenerateEqualsAndHash. */
            public javaGenerateEqualsAndHash: boolean;

            /** FileOptions javaStringCheckUtf8. */
            public javaStringCheckUtf8: boolean;

            /** FileOptions optimizeFor. */
            public optimizeFor: google.protobuf.FileOptions.OptimizeMode;

            /** FileOptions goPackage. */
            public goPackage: string;

            /** FileOptions ccGenericServices. */
            public ccGenericServices: boolean;

            /** FileOptions javaGenericServices. */
            public javaGenericServices: boolean;

            /** FileOptions pyGenericServices. */
            public pyGenericServices: boolean;

            /** FileOptions deprecated. */
            public deprecated: boolean;

            /** FileOptions ccEnableArenas. */
            public ccEnableArenas: boolean;

            /** FileOptions objcClassPrefix. */
            public objcClassPrefix: string;

            /** FileOptions csharpNamespace. */
            public csharpNamespace: string;

            /** FileOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified FileOptions message. Does not implicitly {@link google.protobuf.FileOptions.verify|verify} messages.
             * @param message FileOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IFileOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FileOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FileOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.FileOptions;
        }

        namespace FileOptions {

            /** OptimizeMode enum. */
            enum OptimizeMode {
                SPEED = 1,
                CODE_SIZE = 2,
                LITE_RUNTIME = 3
            }
        }

        /** Properties of a MessageOptions. */
        interface IMessageOptions {

            /** MessageOptions messageSetWireFormat */
            messageSetWireFormat?: (boolean|null);

            /** MessageOptions noStandardDescriptorAccessor */
            noStandardDescriptorAccessor?: (boolean|null);

            /** MessageOptions deprecated */
            deprecated?: (boolean|null);

            /** MessageOptions mapEntry */
            mapEntry?: (boolean|null);

            /** MessageOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents a MessageOptions. */
        class MessageOptions implements IMessageOptions {

            /**
             * Constructs a new MessageOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IMessageOptions);

            /** MessageOptions messageSetWireFormat. */
            public messageSetWireFormat: boolean;

            /** MessageOptions noStandardDescriptorAccessor. */
            public noStandardDescriptorAccessor: boolean;

            /** MessageOptions deprecated. */
            public deprecated: boolean;

            /** MessageOptions mapEntry. */
            public mapEntry: boolean;

            /** MessageOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified MessageOptions message. Does not implicitly {@link google.protobuf.MessageOptions.verify|verify} messages.
             * @param message MessageOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IMessageOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MessageOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MessageOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.MessageOptions;
        }

        /** Properties of a FieldOptions. */
        interface IFieldOptions {

            /** FieldOptions ctype */
            ctype?: (google.protobuf.FieldOptions.CType|null);

            /** FieldOptions packed */
            packed?: (boolean|null);

            /** FieldOptions jstype */
            jstype?: (google.protobuf.FieldOptions.JSType|null);

            /** FieldOptions lazy */
            lazy?: (boolean|null);

            /** FieldOptions deprecated */
            deprecated?: (boolean|null);

            /** FieldOptions weak */
            weak?: (boolean|null);

            /** FieldOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);

            /** FieldOptions .pointer */
            ".pointer"?: (PointerType|null);

            /** FieldOptions .arraySize */
            ".arraySize"?: (number|null);
        }

        /** Represents a FieldOptions. */
        class FieldOptions implements IFieldOptions {

            /**
             * Constructs a new FieldOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IFieldOptions);

            /** FieldOptions ctype. */
            public ctype: google.protobuf.FieldOptions.CType;

            /** FieldOptions packed. */
            public packed: boolean;

            /** FieldOptions jstype. */
            public jstype: google.protobuf.FieldOptions.JSType;

            /** FieldOptions lazy. */
            public lazy: boolean;

            /** FieldOptions deprecated. */
            public deprecated: boolean;

            /** FieldOptions weak. */
            public weak: boolean;

            /** FieldOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /** FieldOptions .pointer. */
            public pointer: PointerType;

            /** FieldOptions .arraySize. */
            public arraySize: number;

            /**
             * Encodes the specified FieldOptions message. Does not implicitly {@link google.protobuf.FieldOptions.verify|verify} messages.
             * @param message FieldOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IFieldOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a FieldOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns FieldOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.FieldOptions;
        }

        namespace FieldOptions {

            /** CType enum. */
            enum CType {
                STRING = 0,
                CORD = 1,
                STRING_PIECE = 2
            }

            /** JSType enum. */
            enum JSType {
                JS_NORMAL = 0,
                JS_STRING = 1,
                JS_NUMBER = 2
            }
        }

        /** Properties of an OneofOptions. */
        interface IOneofOptions {

            /** OneofOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents an OneofOptions. */
        class OneofOptions implements IOneofOptions {

            /**
             * Constructs a new OneofOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IOneofOptions);

            /** OneofOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified OneofOptions message. Does not implicitly {@link google.protobuf.OneofOptions.verify|verify} messages.
             * @param message OneofOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IOneofOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an OneofOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns OneofOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.OneofOptions;
        }

        /** Properties of an EnumOptions. */
        interface IEnumOptions {

            /** EnumOptions allowAlias */
            allowAlias?: (boolean|null);

            /** EnumOptions deprecated */
            deprecated?: (boolean|null);

            /** EnumOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents an EnumOptions. */
        class EnumOptions implements IEnumOptions {

            /**
             * Constructs a new EnumOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IEnumOptions);

            /** EnumOptions allowAlias. */
            public allowAlias: boolean;

            /** EnumOptions deprecated. */
            public deprecated: boolean;

            /** EnumOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified EnumOptions message. Does not implicitly {@link google.protobuf.EnumOptions.verify|verify} messages.
             * @param message EnumOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IEnumOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EnumOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EnumOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.EnumOptions;
        }

        /** Properties of an EnumValueOptions. */
        interface IEnumValueOptions {

            /** EnumValueOptions deprecated */
            deprecated?: (boolean|null);

            /** EnumValueOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents an EnumValueOptions. */
        class EnumValueOptions implements IEnumValueOptions {

            /**
             * Constructs a new EnumValueOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IEnumValueOptions);

            /** EnumValueOptions deprecated. */
            public deprecated: boolean;

            /** EnumValueOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified EnumValueOptions message. Does not implicitly {@link google.protobuf.EnumValueOptions.verify|verify} messages.
             * @param message EnumValueOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IEnumValueOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EnumValueOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EnumValueOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.EnumValueOptions;
        }

        /** Properties of a ServiceOptions. */
        interface IServiceOptions {

            /** ServiceOptions deprecated */
            deprecated?: (boolean|null);

            /** ServiceOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents a ServiceOptions. */
        class ServiceOptions implements IServiceOptions {

            /**
             * Constructs a new ServiceOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IServiceOptions);

            /** ServiceOptions deprecated. */
            public deprecated: boolean;

            /** ServiceOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified ServiceOptions message. Does not implicitly {@link google.protobuf.ServiceOptions.verify|verify} messages.
             * @param message ServiceOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IServiceOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ServiceOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ServiceOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.ServiceOptions;
        }

        /** Properties of a MethodOptions. */
        interface IMethodOptions {

            /** MethodOptions deprecated */
            deprecated?: (boolean|null);

            /** MethodOptions uninterpretedOption */
            uninterpretedOption?: (google.protobuf.IUninterpretedOption[]|null);
        }

        /** Represents a MethodOptions. */
        class MethodOptions implements IMethodOptions {

            /**
             * Constructs a new MethodOptions.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IMethodOptions);

            /** MethodOptions deprecated. */
            public deprecated: boolean;

            /** MethodOptions uninterpretedOption. */
            public uninterpretedOption: google.protobuf.IUninterpretedOption[];

            /**
             * Encodes the specified MethodOptions message. Does not implicitly {@link google.protobuf.MethodOptions.verify|verify} messages.
             * @param message MethodOptions message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IMethodOptions, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MethodOptions message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MethodOptions
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.MethodOptions;
        }

        /** Properties of an UninterpretedOption. */
        interface IUninterpretedOption {

            /** UninterpretedOption name */
            name?: (google.protobuf.UninterpretedOption.INamePart[]|null);

            /** UninterpretedOption identifierValue */
            identifierValue?: (string|null);

            /** UninterpretedOption positiveIntValue */
            positiveIntValue?: (number|Long|null);

            /** UninterpretedOption negativeIntValue */
            negativeIntValue?: (number|Long|null);

            /** UninterpretedOption doubleValue */
            doubleValue?: (number|null);

            /** UninterpretedOption stringValue */
            stringValue?: (Uint8Array|null);

            /** UninterpretedOption aggregateValue */
            aggregateValue?: (string|null);
        }

        /** Represents an UninterpretedOption. */
        class UninterpretedOption implements IUninterpretedOption {

            /**
             * Constructs a new UninterpretedOption.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IUninterpretedOption);

            /** UninterpretedOption name. */
            public name: google.protobuf.UninterpretedOption.INamePart[];

            /** UninterpretedOption identifierValue. */
            public identifierValue: string;

            /** UninterpretedOption positiveIntValue. */
            public positiveIntValue: (number|Long);

            /** UninterpretedOption negativeIntValue. */
            public negativeIntValue: (number|Long);

            /** UninterpretedOption doubleValue. */
            public doubleValue: number;

            /** UninterpretedOption stringValue. */
            public stringValue: Uint8Array;

            /** UninterpretedOption aggregateValue. */
            public aggregateValue: string;

            /**
             * Encodes the specified UninterpretedOption message. Does not implicitly {@link google.protobuf.UninterpretedOption.verify|verify} messages.
             * @param message UninterpretedOption message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IUninterpretedOption, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an UninterpretedOption message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UninterpretedOption
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.UninterpretedOption;
        }

        namespace UninterpretedOption {

            /** Properties of a NamePart. */
            interface INamePart {

                /** NamePart namePart */
                namePart: string;

                /** NamePart isExtension */
                isExtension: boolean;
            }

            /** Represents a NamePart. */
            class NamePart implements INamePart {

                /**
                 * Constructs a new NamePart.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: google.protobuf.UninterpretedOption.INamePart);

                /** NamePart namePart. */
                public namePart: string;

                /** NamePart isExtension. */
                public isExtension: boolean;

                /**
                 * Encodes the specified NamePart message. Does not implicitly {@link google.protobuf.UninterpretedOption.NamePart.verify|verify} messages.
                 * @param message NamePart message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: google.protobuf.UninterpretedOption.INamePart, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a NamePart message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns NamePart
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.UninterpretedOption.NamePart;
            }
        }

        /** Properties of a SourceCodeInfo. */
        interface ISourceCodeInfo {

            /** SourceCodeInfo location */
            location?: (google.protobuf.SourceCodeInfo.ILocation[]|null);
        }

        /** Represents a SourceCodeInfo. */
        class SourceCodeInfo implements ISourceCodeInfo {

            /**
             * Constructs a new SourceCodeInfo.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.ISourceCodeInfo);

            /** SourceCodeInfo location. */
            public location: google.protobuf.SourceCodeInfo.ILocation[];

            /**
             * Encodes the specified SourceCodeInfo message. Does not implicitly {@link google.protobuf.SourceCodeInfo.verify|verify} messages.
             * @param message SourceCodeInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.ISourceCodeInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a SourceCodeInfo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns SourceCodeInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.SourceCodeInfo;
        }

        namespace SourceCodeInfo {

            /** Properties of a Location. */
            interface ILocation {

                /** Location path */
                path?: (number[]|null);

                /** Location span */
                span?: (number[]|null);

                /** Location leadingComments */
                leadingComments?: (string|null);

                /** Location trailingComments */
                trailingComments?: (string|null);

                /** Location leadingDetachedComments */
                leadingDetachedComments?: (string[]|null);
            }

            /** Represents a Location. */
            class Location implements ILocation {

                /**
                 * Constructs a new Location.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: google.protobuf.SourceCodeInfo.ILocation);

                /** Location path. */
                public path: number[];

                /** Location span. */
                public span: number[];

                /** Location leadingComments. */
                public leadingComments: string;

                /** Location trailingComments. */
                public trailingComments: string;

                /** Location leadingDetachedComments. */
                public leadingDetachedComments: string[];

                /**
                 * Encodes the specified Location message. Does not implicitly {@link google.protobuf.SourceCodeInfo.Location.verify|verify} messages.
                 * @param message Location message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: google.protobuf.SourceCodeInfo.ILocation, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Location message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Location
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.SourceCodeInfo.Location;
            }
        }

        /** Properties of a GeneratedCodeInfo. */
        interface IGeneratedCodeInfo {

            /** GeneratedCodeInfo annotation */
            annotation?: (google.protobuf.GeneratedCodeInfo.IAnnotation[]|null);
        }

        /** Represents a GeneratedCodeInfo. */
        class GeneratedCodeInfo implements IGeneratedCodeInfo {

            /**
             * Constructs a new GeneratedCodeInfo.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IGeneratedCodeInfo);

            /** GeneratedCodeInfo annotation. */
            public annotation: google.protobuf.GeneratedCodeInfo.IAnnotation[];

            /**
             * Encodes the specified GeneratedCodeInfo message. Does not implicitly {@link google.protobuf.GeneratedCodeInfo.verify|verify} messages.
             * @param message GeneratedCodeInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IGeneratedCodeInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a GeneratedCodeInfo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns GeneratedCodeInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.GeneratedCodeInfo;
        }

        namespace GeneratedCodeInfo {

            /** Properties of an Annotation. */
            interface IAnnotation {

                /** Annotation path */
                path?: (number[]|null);

                /** Annotation sourceFile */
                sourceFile?: (string|null);

                /** Annotation begin */
                begin?: (number|null);

                /** Annotation end */
                end?: (number|null);
            }

            /** Represents an Annotation. */
            class Annotation implements IAnnotation {

                /**
                 * Constructs a new Annotation.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: google.protobuf.GeneratedCodeInfo.IAnnotation);

                /** Annotation path. */
                public path: number[];

                /** Annotation sourceFile. */
                public sourceFile: string;

                /** Annotation begin. */
                public begin: number;

                /** Annotation end. */
                public end: number;

                /**
                 * Encodes the specified Annotation message. Does not implicitly {@link google.protobuf.GeneratedCodeInfo.Annotation.verify|verify} messages.
                 * @param message Annotation message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: google.protobuf.GeneratedCodeInfo.IAnnotation, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes an Annotation message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Annotation
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.GeneratedCodeInfo.Annotation;
            }
        }

        /** Properties of a Duration. */
        interface IDuration {

            /** Duration seconds */
            seconds?: (number|Long|null);

            /** Duration nanos */
            nanos?: (number|null);
        }

        /** Represents a Duration. */
        class Duration implements IDuration {

            /**
             * Constructs a new Duration.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IDuration);

            /** Duration seconds. */
            public seconds: (number|Long);

            /** Duration nanos. */
            public nanos: number;

            /**
             * Encodes the specified Duration message. Does not implicitly {@link google.protobuf.Duration.verify|verify} messages.
             * @param message Duration message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IDuration, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Duration message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Duration
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.Duration;
        }
    }
}

/** PointerType enum. */
export enum PointerType {
    NONE = 0,
    RAW = 1,
    SHARED = 2,
    UNIQUE = 3
}
