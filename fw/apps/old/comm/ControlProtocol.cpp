//
// Created by zhouj on 2023/4/1.
//

#include "ControlProtocol.hpp"
#include "system.hpp"

namespace wibot::motor {

void ControlProtocol::doRxWork() {
    _stream.open();

    BUFFER(frameBuf, 16);
    MessageFrame frame(frameBuf);

    while (true) {
        auto rst = _stream.readSync(frame, TIMEOUT_FOREVER);
        if (rst == Result::kOk) {
            auto cmd = static_cast<CommandType>(frame.getCommand().data[0]);
            switch (cmd) {
                case CommandType::PID_CURRENT_GET_REQ: {
                    this->_reqState.pidCurrent = true;
                    break;
                }
                case CommandType::PID_SPEED_GET_REQ: {
                    this->_reqState.pidSpeed = true;
                    break;
                }
                case CommandType::PID_POSITION_GET_REQ: {
                    this->_reqState.pidPosition = true;
                    break;
                }
                case CommandType::PID_CURRENT_SET_REQ:
                case CommandType::PID_SPEED_SET_REQ:
                case CommandType::PID_POSITION_SET_REQ: {
                    auto ctn = frame.getContent();
                    _doSetPid(cmd, ctn.getFloat(0), ctn.getFloat(4), ctn.getFloat(8));
                    break;
                }
                case CommandType::MODE_STOP_REQ: {
                    _doStop();
                    break;
                }
                case CommandType::MODE_CALIBRATE_REQ: {
                    _doCalibrate();
                    break;
                }
                case CommandType::MODE_OPEN_LOOP_REQ: {
                    auto ctn = frame.getContent();
                    _doOpenLoop(ctn.getFloat(0), ctn.getFloat(4));
                    break;
                }
                case CommandType::MODE_CURRENT_CLOSE_LOOP_REQ: {
                    auto ctn = frame.getContent();
                    _doCurrentCloseLoop(ctn.getFloat(0), ctn.getFloat(4));
                    break;
                }
                case CommandType::MODE_SPEED_CLOSE_LOOP_REQ: {
                    auto ctn = frame.getContent();
                    _doSpeedLoop(ctn.getFloat(0));
                    break;
                }
                case CommandType::MODE_POSITION_CLOSE_LOOP_REQ: {
                    auto ctn = frame.getContent();
                    _doPositionLoop(ctn.getFloat(0));
                    break;
                }

                case CommandType::MONITOR_UBUS_START_REQ: {
                    _monitorState.ubus = true;
                    break;
                }
                case CommandType::MONITOR_UBUS_STOP_REQ: {
                    _monitorState.ubus = false;
                    break;
                }

                case CommandType::MONITOR_IBUS_START_REQ: {
                    _monitorState.ibus = true;
                    break;
                }
                case CommandType::MONITOR_IBUS_STOP_REQ: {
                    _monitorState.ibus = false;
                    break;
                }

                case CommandType::MONITOR_SPDM_START_REQ: {
                    _monitorState.spdm = true;
                    break;
                }
                case CommandType::MONITOR_SPDM_STOP_REQ: {
                    _monitorState.spdm = false;
                    break;
                }

                case CommandType::MONITOR_SPDMREF_START_REQ: {
                    _monitorState.spdm_ref = true;
                    break;
                }
                case CommandType::MONITOR_SPDMREF_STOP_REQ: {
                    _monitorState.spdm_ref = false;
                    break;
                }

                case CommandType::MONITOR_POSM_START_REQ: {
                    _monitorState.posm = true;
                    break;
                }
                case CommandType::MONITOR_POSM_STOP_REQ: {
                    _monitorState.posm = false;
                    break;
                }

                case CommandType::MONITOR_POSMREF_START_REQ: {
                    _monitorState.posm_ref = true;
                    break;
                }
                case CommandType::MONITOR_POSMREF_STOP_REQ: {
                    _monitorState.posm_ref = false;
                    break;
                }

                case CommandType::MONITOR_UABC_START_REQ: {
                    _monitorState.uabc = true;
                    break;
                }
                case CommandType::MONITOR_UABC_STOP_REQ: {
                    _monitorState.uabc = false;
                    break;
                }

                case CommandType::MONITOR_DABCREF_START_REQ: {
                    _monitorState.dabc_ref = true;
                    break;
                }
                case CommandType::MONITOR_DABCREF_STOP_REQ: {
                    _monitorState.dabc_ref = false;
                    break;
                }

                case CommandType::MONITOR_IABC_START_REQ: {
                    _monitorState.iabc = true;
                    break;
                }
                case CommandType::MONITOR_IABC_STOP_REQ: {
                    _monitorState.iabc = false;
                    break;
                }

                    //            case CommandType::MONITOR_IABCREF_START_REQ: {
                    //                _monitorState.iabc_ref = true;
                    //                break;
                    //            }
                    //            case CommandType::MONITOR_IABCREF_STOP_REQ: {
                    //                _monitorState.iabc_ref = false;
                    //                break;
                    //            }
                    //
                    //            case CommandType::MONITOR_UDQ_START_REQ: {
                    //                _monitorState.udq = true;
                    //                break;
                    //            }
                    //            case CommandType::MONITOR_UDQ_STOP_REQ: {
                    //                _monitorState.udq = false;
                    //                break;
                    //            }

                case CommandType::MONITOR_UDQREF_START_REQ: {
                    _monitorState.udq_ref = true;
                    break;
                }
                case CommandType::MONITOR_UDQREF_STOP_REQ: {
                    _monitorState.udq_ref = false;
                    break;
                }

                case CommandType::MONITOR_IDQ_START_REQ: {
                    _monitorState.idq = true;
                    break;
                }
                case CommandType::MONITOR_IDQ_STOP_REQ: {
                    _monitorState.idq = false;
                    break;
                }

                case CommandType::MONITOR_IDQREF_START_REQ: {
                    _monitorState.idq_ref = true;
                    break;
                }
                case CommandType::MONITOR_IDQREF_STOP_REQ: {
                    _monitorState.idq_ref = false;
                    break;
                }

                case CommandType::MONITOR_SEC_START_REQ: {
                    _monitorState.sec = true;
                    break;
                }
                case CommandType::MONITOR_SEC_STOP_REQ: {
                    _monitorState.sec = false;
                    break;
                }

                case CommandType::MONITOR_SECREF_START_REQ: {
                    _monitorState.sec_ref = true;
                    break;
                }
                case CommandType::MONITOR_SECREF_STOP_REQ: {
                    _monitorState.sec_ref = false;
                    break;
                }

                case CommandType::MONITOR_SPDE_START_REQ: {
                    _monitorState.spde = true;
                    break;
                }
                case CommandType::MONITOR_SPDE_STOP_REQ: {
                    _monitorState.spde = false;
                    break;
                }

                    //            case CommandType::MONITOR_SPDEREF_START_REQ: {
                    //                _monitorState.spde_ref = true;
                    //                break;
                    //            }
                    //            case CommandType::MONITOR_SPDEREF_STOP_REQ: {
                    //                _monitorState.spde_ref = false;
                    //                break;
                    //            }

                case CommandType::MONITOR_POSE_START_REQ: {
                    _monitorState.pose = true;
                    break;
                }
                case CommandType::MONITOR_POSE_STOP_REQ: {
                    _monitorState.pose = false;
                    break;
                }

                    //            case CommandType::MONITOR_POSEREF_START_REQ: {
                    //                _monitorState.pose_ref = true;
                    //                break;
                    //            }
                    //            case CommandType::MONITOR_POSEREF_STOP_REQ: {
                    //                _monitorState.pose_ref = false;
                    //                break;
                    //            }

                case CommandType::MONITOR_IBUSREF_START_REQ: {
                    _monitorState.ibus_ref = true;
                    break;
                }
                case CommandType::MONITOR_IBUSREF_STOP_REQ: {
                    _monitorState.ibus_ref = false;
                    break;
                }

                case CommandType::MONITOR_DBUSREF_START_REQ: {
                    _monitorState.dbus_ref = true;
                    break;
                }
                case CommandType::MONITOR_DBUSREF_STOP_REQ: {
                    _monitorState.dbus_ref = false;
                    break;
                }

                case CommandType::MONITOR_SWREF_START_REQ: {
                    _monitorState.sw_ref = true;
                    break;
                }
                case CommandType::MONITOR_SWREF_STOP_REQ: {
                    _monitorState.sw_ref = false;
                    break;
                }

                case CommandType::MONITOR_DSPLREF_START_REQ: {
                    _monitorState.dsample_ref = true;
                    break;
                }
                case CommandType::MONITOR_DSPLREF_STOP_REQ: {
                    _monitorState.dsample_ref = false;
                    break;
                }

                default:
                    break;
            }
        }
    }
}
Buffer ControlProtocol::_doGetPid(PidType type) {
    BUFFER(buffer, 16);
    MessageFrame frame(buffer);

    Vector3f pid;
    switch (type) {
        case PidType::current:
            pid = _focCtrl.getCurrentPid();
            frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::PID_CURRENT_GET_RESP));
            break;
        case PidType::speed:
            //            pid = _focCtrl.getCurrentPid();
            //            frame.getCommand().setUint8(0,
            //            static_cast<uint8_t>(CommandType::PID_SPEED_GET_RESP));
            break;
        case PidType::position:
            //            pid = _focCtrl.getCurrentPid();
            //            frame.getCommand().setUint8(0,
            //                                        static_cast<uint8_t>(CommandType::PID_POSITION_GET_RESP));
            break;
    }

    auto ctn = frame.getContent();
    ctn.setFloat(0, pid.v1, Endian::kBig);
    ctn.setFloat(4, pid.v2, Endian::kBig);
    ctn.setFloat(8, pid.v3, Endian::kBig);
    return frame.getWholeBuffer();
}

void ControlProtocol::_doSetPid(CommandType type, float p, float i, float d) {
    switch (type) {
        case CommandType::PID_CURRENT_GET_REQ: {
            _focCtrl.setCurrentPid(p, i, d);
            break;
        }
        case CommandType::PID_SPEED_SET_REQ: {
            //_focCtrl.set_speed_pid(_motor, p, i, d);
            break;
        }
        case CommandType::PID_POSITION_SET_REQ: {
            //_focCtrl.set_position_pid(_motor, p, i, d);
            break;
        }
        default:
            break;
    }
}
void ControlProtocol::_doStop() {
    FocCommand cmd(ControlMode::kStop);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doCalibrate() {
    FocCommand cmd(ControlMode::kCalibrate);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doOpenLoop(float ud, float uq) {
    FocCommand cmd(ControlMode::kOpenLoop);
    cmd.voltage = Vector2f(ud, uq);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doCurrentCloseLoop(float id, float iq) {
    FocCommand cmd(ControlMode::kCurrent);
    cmd.current = Vector2f(id, iq);
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doSpeedLoop(float ref) {
    FocCommand cmd(ControlMode::kSpeed);
    cmd.speed = ref;
    _focCtrl.set_command(_motor, cmd);
}
void ControlProtocol::_doPositionLoop(float ref) {
    FocCommand cmd(ControlMode::kPosition);
    cmd.position = ref;
    _focCtrl.set_command(_motor, cmd);
}
Buffer ControlProtocol::_doMonitor() {
    // prefix:2, cmd:1, length:1, (state:4, tick:4, data...), crc:1
    uint32_t ctnLen = 8;
    auto     state  = _monitorState;
    if (state.ubus) {
        ctnLen += 4;
    }
    if (state.ibus) {
        ctnLen += 4;
    }
    if (state.spdm) {
        ctnLen += 4;
    }
    if (state.spdm_ref) {
        ctnLen += 4;
    }
    if (state.posm) {
        ctnLen += 4;
    }
    if (state.posm_ref) {
        ctnLen += 4;
    }
    if (state.uabc) {
        ctnLen += 12;
    }
    if (state.dabc_ref) {
        ctnLen += 12;
    }
    if (state.iabc) {
        ctnLen += 12;
    }
    //    if (state.iabc_ref) {
    //        ctnLen += 12;
    //    }
    //    if (state.udq) {
    //        ctnLen += 8;
    //    }
    if (state.udq_ref) {
        ctnLen += 8;
    }
    if (state.idq) {
        ctnLen += 8;
    }
    if (state.idq_ref) {
        ctnLen += 8;
    }
    if (state.sec) {
        ctnLen += 1;
    }
    if (state.sec_ref) {
        ctnLen += 1;
    }
    if (state.spde) {
        ctnLen += 4;
    }
    //    if (state.spde_ref) {
    //        ctnLen += 4;
    //    }
    if (state.pose) {
        ctnLen += 4;
    }
    //    if (state.pose_ref) {
    //        ctnLen += 4;
    //    }
    if (state.ibus_ref) {
        ctnLen += 4;
    }
    if (state.sw_ref) {
        ctnLen += 1;
    }
    if (state.dsample_ref) {
        ctnLen += 4;
    }

    auto cmd = toUnderlying(CommandType::MONITOR_RESP);

    MessageFrame frame(_tx_buffer, schema, &cmd, ctnLen);
    frame.getCommand().setUint8(0, static_cast<uint8_t>(CommandType::MONITOR_RESP));
    auto          ctn = frame.getContent();
    ctn.resetIndex();
    ctn.setUint32(System::getTickMs());
    if (state.ubus) {
        ctn.setFloat(_motor.state.uBus);
    }
    if (state.ibus) {
        ctn.setFloat(_motor.state.iBus);
    }
    if (state.spdm) {
        ctn.setFloat(_motor.state.speed.v2);
    }
    if (state.spdm_ref) {
        ctn.setFloat(_motor.reference.speed);
    }
    if (state.posm) {
        ctn.setFloat(_motor.state.position.v2);
    }
    if (state.posm_ref) {
        ctn.setFloat(_motor.reference.position);
    }
    if (state.uabc) {
        ctn.setFloat(_motor.state.uAbc.v1);
        ctn.setFloat(_motor.state.uAbc.v2);
        ctn.setFloat(_motor.state.uAbc.v3);
    }
    if (state.dabc_ref) {
        ctn.setFloat(_motor.reference.dAbc.v1);
        ctn.setFloat(_motor.reference.dAbc.v2);
        ctn.setFloat(_motor.reference.dAbc.v3);
    }
    if (state.iabc) {
        ctn.setFloat(_motor.state.iAbc.v1);
        ctn.setFloat(_motor.state.iAbc.v2);
        ctn.setFloat(_motor.state.iAbc.v3);
    }
    //    if (state.iabc_ref) {
    //        bs.setFloat(.0f);
    //        bs.setFloat(.0f);
    //        bs.setFloat(.0f);
    //    }
    //    if (state.udq) {
    //        bs.setFloat(_motor.state..v1);
    //        bs.setFloat(_motor.state.i_abc.v2);
    //        bs.setFloat(_motor.state.i_abc.v3);
    //    }
    if (state.udq_ref) {
        ctn.setFloat(_motor.reference.uDq.v1);
        ctn.setFloat(_motor.reference.uDq.v2);
    }
    if (state.idq) {
        ctn.setFloat(_motor.state.iDq.v1);
        ctn.setFloat(_motor.state.iDq.v2);
    }
    if (state.idq_ref) {
        ctn.setFloat(_motor.reference.iDq.v1);
        ctn.setFloat(_motor.reference.iDq.v2);
    }
    if (state.sec) {
        ctn.setUint8(_motor.state.section);
    }
    if (state.sec_ref) {
        ctn.setUint8(_motor.reference.section);
    }
    if (state.spde) {
        ctn.setFloat(_motor.state.speed.v1);
    }
    //    if (state.spde_ref) {
    //        ctnLen += 4;
    //    }
    if (state.pose) {
        ctn.setFloat(_motor.state.position.v1);
    }
    //    if (state.pose_ref) {
    //        ctnLen += 4;
    //    }
//    if (state.ibus_ref) {
//        ctn.setFloat(_motor.reference.iBus);
//    }
//    if (state.sw_ref) {
//        ctn.setUint8(_motor.reference.sw_channel);
//    }
//    if (state.dsample_ref) {
//        ctn.setFloat(_motor.reference.dSample);
//    }
    return frame.getWholeBuffer();
}
Buffer ControlProtocol::doTxWork() {
    if (_reqState.pidCurrent) {
        _reqState.pidCurrent = false;
        return _doGetPid(PidType::current);
    }
    if (_reqState.pidSpeed) {
        _reqState.pidSpeed = false;
        return _doGetPid(PidType::speed);
    }
    if (_reqState.pidPosition) {
        _reqState.pidPosition = false;
        return _doGetPid(PidType::position);
    }
    return _doMonitor();
}

void ControlProtocol::startRx() {
    doRxWork();
}

void ControlProtocol::startTx() {
    // do {
    //     Buffer data = doTxWork();

    //     Result txRst;  //_stream.writeAsync(data.data, data.size, _txWaitHandle);
    //     if (txRst == Result::kOk) {
    //         txRst = _txWaitHandle.wait(TIMEOUT_FOREVER);
    //         if (txRst == Result::kGeneralError) {
    //             _resetUart();
    //         } else if (txRst == Result::kOk) {
    //             doRxWork();
    //         }
    //     }
    //     Thread::sleep(1);
    // } while (true);
};

}  // namespace wibot::motor
