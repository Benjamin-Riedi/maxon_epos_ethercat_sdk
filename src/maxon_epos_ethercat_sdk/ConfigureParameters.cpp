// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#include <array>
#include <cmath>
#include <thread>
#include <vector>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon {
bool Maxon::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  const auto timeout = configuration_.configRunSdoVerifyTimeout;

  const auto mapPdo = [&](uint16_t assignmentIndex,
                          uint16_t mappingIndex,
                          const std::vector<uint32_t>& objects) {
    bool success = true;
    uint8_t subIndex = 0;

    success &= sdoVerifyWrite(assignmentIndex, 0x00, false,
                              static_cast<uint8_t>(0), timeout);
    success &= sdoVerifyWrite(mappingIndex, 0x00, false,
                              static_cast<uint8_t>(0), timeout);
    success &= sdoVerifyWrite(assignmentIndex, 0x01, false, mappingIndex,
                              timeout);

    for (const auto& objectIndex : objects) {
      subIndex += 1;
      success &= sdoVerifyWrite(mappingIndex, subIndex, false, objectIndex,
                                timeout);
    }

    success &= sdoVerifyWrite(mappingIndex, 0x00, false, subIndex, timeout);
    success &= sdoVerifyWrite(assignmentIndex, 0x00, false,
                              static_cast<uint8_t>(1), timeout);
    return success;
  };

  const auto mapSplitPdo = [&](uint16_t assignmentIndex,
                               uint16_t mappingIndex3,
                               const std::vector<uint32_t>& objects3,
                               uint16_t mappingIndex4,
                               const std::vector<uint32_t>& objects4) {
    bool success = true;
    uint8_t subIndex = 0;

    success &= sdoVerifyWrite(assignmentIndex, 0x00, false,
                              static_cast<uint8_t>(0), timeout);
    success &= sdoVerifyWrite(mappingIndex3, 0x00, false,
                              static_cast<uint8_t>(0), timeout);
    success &= sdoVerifyWrite(mappingIndex4, 0x00, false,
                              static_cast<uint8_t>(0), timeout);
    success &= sdoVerifyWrite(assignmentIndex, 0x01, false, mappingIndex3,
                              timeout);
    success &= sdoVerifyWrite(assignmentIndex, 0x02, false, mappingIndex4,
                              timeout);

    subIndex = 0;
    for (const auto& objectIndex : objects3) {
      subIndex += 1;
      success &= sdoVerifyWrite(mappingIndex3, subIndex, false, objectIndex,
                                timeout);
    }
    success &= sdoVerifyWrite(mappingIndex3, 0x00, false, subIndex, timeout);

    subIndex = 0;
    for (const auto& objectIndex : objects4) {
      subIndex += 1;
      success &= sdoVerifyWrite(mappingIndex4, subIndex, false, objectIndex,
                                timeout);
    }
    success &= sdoVerifyWrite(mappingIndex4, 0x00, false, subIndex, timeout);

    success &= sdoVerifyWrite(assignmentIndex, 0x00, false,
                              static_cast<uint8_t>(2), timeout);
    return success;
  };

  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Standard Mode");
            rxSuccess &= mapPdo(OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
                                                    std::vector<uint32_t>{});
      break;
    }
    case RxPdoTypeEnum::RxPdoCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Position Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Toruqe Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoCSV: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Veloctity Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Toruqe/Position Mixed Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
          << "Cyclic Synchronous Toruqe/Position/Velocity Mixed Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSPCSVHM: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
          << "Cyclic Synchronous Toruqe/Position/Velocity/Homing Mixed Mode");
      rxSuccess &= mapSplitPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) |
                  sizeof(uint16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8},
          OD_INDEX_RX_PDO_MAPPING_4,
          std::vector<uint32_t>{
              (OD_INDEX_HOMING_METHOD << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8,
              (OD_INDEX_HOMING_SPEEDS << 16) | (0x01 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOMING_SPEEDS << 16) | (0x02 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOMING_ACCELERATION << 16) | (0x00 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOME_OFFSET << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_HOME_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CURRENT_THRESHOLD << 16) | (0x00 << 8) |
                  sizeof(uint16_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoPVM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Velocity Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8});
      break;
    }
    case RxPdoTypeEnum::RxPdoHM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Homing Mode");
      rxSuccess &= mapPdo(
          OD_INDEX_RX_PDO_ASSIGNMENT, OD_INDEX_RX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
              (OD_INDEX_HOMING_METHOD << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8,
              (OD_INDEX_HOMING_SPEEDS << 16) | (0x01 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOMING_SPEEDS << 16) | (0x02 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOMING_ACCELERATION << 16) | (0x00 << 8) |
                  sizeof(uint32_t) * 8,
              (OD_INDEX_HOME_OFFSET << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_HOME_POSITION << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_CURRENT_THRESHOLD << 16) | (0x00 << 8) |
                  sizeof(uint16_t) * 8,
              (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
                  sizeof(int8_t) * 8});
      break;
    }
    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "RxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map unimplemented "
          "RxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Standard Mode");
      txSuccess &= mapPdo(OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
                          {});
      break;
    }
    case TxPdoTypeEnum::TxPdoCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Position Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoCSV: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Velocity Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSP: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque/Position Mixed Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
          << "Cyclic Synchronous Torque/Position/Velocity Mixed Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSPCSVHM: {
      MELO_INFO_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
          << "Cyclic Synchronous Torque/Position/Velocity/Homing Mixed Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int16_t) * 8,
              (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8,
              (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoPVM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Profile Velocity Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
              (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) |
                  sizeof(int32_t) * 8});
      break;
    }
    case TxPdoTypeEnum::TxPdoHM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Homing Mode");
      txSuccess &= mapPdo(
          OD_INDEX_TX_PDO_ASSIGNMENT, OD_INDEX_TX_PDO_MAPPING_3,
          std::vector<uint32_t>{
              (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8});
      break;
    }
    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "TxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map undefined "
          "TxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
  }

  return (txSuccess && rxSuccess);
}

bool Maxon::configParam() {
  bool configSuccess = true;
  uint32_t maxMotorSpeed;
  uint32_t maxProfileVelocity;
  uint32_t maxGearInputSpeed;
  uint32_t nominalCurrent;
  uint32_t maxCurrent;
  uint32_t torqueConstant;
  uint32_t currentPGain;
  uint32_t currentIGain;
  uint32_t positionPGain;
  uint32_t positionIGain;
  uint32_t positionDGain;
  uint32_t velocityPGain;
  uint32_t velocityIGain;

  // Set velocity unit to rpm
  uint32_t velocity_unit;
  velocity_unit = 0x00B44700;
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_SI_UNIT_VELOCITY, 0x00, false, velocity_unit,
                     configuration_.configRunSdoVerifyTimeout);

  // maxMotorSpeed = static_cast<uint32_t>(configuration_.workVoltage *
  //                                       configuration_.speedConstant);
  maxMotorSpeed = static_cast<uint32_t>(configuration_.maxMotorSpeed);

  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MAX_MOTOR_SPEED, 0x00, false, maxMotorSpeed,
                     configuration_.configRunSdoVerifyTimeout);

  maxGearInputSpeed = static_cast<uint32_t>(configuration_.maxGearInputSpeed);

  configSuccess &= sdoVerifyWrite(OD_INDEX_GEAR_DATA, 0x03, false,
                                  maxGearInputSpeed,
                                  configuration_.configRunSdoVerifyTimeout);

  // maxProfileVelocity = static_cast<uint32_t>(configuration_.maxProfileVelocity *
  //                                            60.0 / (2 * M_PI));
  maxProfileVelocity = static_cast<uint32_t>(configuration_.maxProfileVelocity);

  configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_PROFILE_VELOCITY, 0x00, false,
                                  maxProfileVelocity,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x01, false,
                                  configuration_.minPosition);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x02, false,
                                  configuration_.maxPosition);

  nominalCurrent =
      static_cast<uint32_t>(round(1000.0 * configuration_.nominalCurrentA));
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x01, false, nominalCurrent,
                     configuration_.configRunSdoVerifyTimeout);

  maxCurrent =
      static_cast<uint32_t>(round(1000.0 * configuration_.maxCurrentA));
  configSuccess &= sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x02, false, maxCurrent,
                                  configuration_.configRunSdoVerifyTimeout);

  torqueConstant =
      static_cast<uint32_t>(1000000.0 * configuration_.torqueConstantNmA);
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x05, false, torqueConstant,
                     configuration_.configRunSdoVerifyTimeout);

  currentPGain = static_cast<uint32_t>(1000000 * configuration_.currentPGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x01, false,
                                  static_cast<uint32_t>(currentPGain),
                                  configuration_.configRunSdoVerifyTimeout);

  currentIGain = static_cast<uint32_t>(1000 * configuration_.currentIGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x02, false,
                                  static_cast<uint32_t>(currentIGain),
                                  configuration_.configRunSdoVerifyTimeout);

  positionPGain =
      static_cast<uint32_t>(1000000 * configuration_.positionPGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x01, false,
                                  static_cast<uint32_t>(positionPGain),
                                  configuration_.configRunSdoVerifyTimeout);

  positionIGain =
      static_cast<uint32_t>(1000000 * configuration_.positionIGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x02, false,
                                  static_cast<uint32_t>(positionIGain),
                                  configuration_.configRunSdoVerifyTimeout);

  positionDGain =
      static_cast<uint32_t>(1000000 * configuration_.positionDGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x03, false,
                                  static_cast<uint32_t>(positionDGain),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_QUICKSTOP_DECELERATION, 0x00, false,
                                  configuration_.quickStopDecel,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_PROFILE_DECELERATION, 0x00, false,
                                  configuration_.profileDecel,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_FOLLOW_ERROR_WINDOW, 0x00, false,
                                  configuration_.followErrorWindow,
                                  configuration_.configRunSdoVerifyTimeout);

  velocityPGain =
      static_cast<uint32_t>(1000000 * configuration_.velocityPGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x01, false,
                                  static_cast<uint32_t>(velocityPGain),
                                  configuration_.configRunSdoVerifyTimeout);

  velocityIGain =
      static_cast<uint32_t>(1000000 * configuration_.velocityIGainSI);
  configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x02, false,
                                  static_cast<uint32_t>(velocityIGain),
                                  configuration_.configRunSdoVerifyTimeout);

  if (configSuccess) {
    MELO_INFO("Setting configuration parameters succeeded.");
  } else {
    MELO_ERROR("Setting configuration parameters failed.");
  }

  return configSuccess;
}
}  // namespace maxon
