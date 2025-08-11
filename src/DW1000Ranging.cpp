/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */


#include "DW1000Ranging.h"
#include "DW1000Device.h"

DW1000RangingClass DW1000Ranging;


//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];

byte         DW1000RangingClass::_currentAddress[8];
byte         DW1000RangingClass::_currentShortAddress[2];
byte         DW1000RangingClass::_lastSentToShortAddress[2];

byte DW1000RangingClass::_channel;

DW1000Device* DW1000RangingClass::_lastSlotDevices[4];    // Die 4 Devices des aktuellen Slots
uint8_t DW1000RangingClass::_lastSlotDeviceCount = 0;     // wie viele Devices im Slot sind
bool DW1000RangingClass::_slotPollAcksReceived[4];        // Für dieses Slot-Set: ACK erhalten?


volatile uint8_t DW1000RangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t      DW1000RangingClass::_lastDistantDevice    = 0; // TODO short, 8bit?
DW1000Mac    DW1000RangingClass::_globalMac;

//module type (anchor or tag)
int16_t      DW1000RangingClass::_type; // TODO enum?

// message flow state
volatile byte    DW1000RangingClass::_expectedMsgId;

// range filter
volatile boolean DW1000RangingClass::_useRangeFilter = false;
uint16_t DW1000RangingClass::_rangeFilterValue = 15;

// protocol error state
boolean          DW1000RangingClass::_protocolFailed = false;

// timestamps to remember
uint32_t          DW1000RangingClass::timer           = 0;
uint8_t 		  DW1000RangingClass::counterForBlink = 0;
uint32_t          DW1000RangingClass::lastSyncTime    = 0;
uint32_t          DW1000RangingClass::roundTripTime   = 0;

// reset line to the chip
uint8_t   DW1000RangingClass::_RST;
uint8_t   DW1000RangingClass::_SS;
// watchdog and reset period
uint32_t  DW1000RangingClass::_lastActivity;
uint32_t  DW1000RangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t  DW1000RangingClass::_replyDelayTimeUS;
//timer delay
uint32_t  DW1000RangingClass::_timerDelay;
uint32_t  DW1000RangingClass::DEFAULT_TIMER_DELAY;
uint32_t  DW1000RangingClass::DEFAULT_REPLY_DELAY_TIME; //in us

//Here our handlers
void (* DW1000RangingClass::_handleNewRange)(void) = 0;
void (* DW1000RangingClass::_handleBlinkDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleNewDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleInactiveDevice)(DW1000Device*) = 0;

uint32_t DW1000RangingClass::MICROS_TO_MILLIS = 1000;

const uint8_t DW1000RangingClass::kRangeDeviceSize = 17;
const uint8_t DW1000RangingClass::kPollDeviceSize = 4;

//uwb controll
bool DW1000RangingClass::_uwbSlot;
TaskHandle_t DW1000RangingClass::uwbProcessingTaskHandle;
//frameBuffer
UwbFrameBuffers  DW1000RangingClass::buffers;
frame DW1000RangingClass::newTxFrame;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */
void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ, const uint32_t Default_Timer_Delay, const uint32_t Default_Replay_Delay_Time) {
	// reset line to the chip
	_RST              = myRST;
	_SS               = mySS;
	_resetPeriod      = DEFAULT_RESET_PERIOD;

	// reply times (same on both sides for symm. ranging)
	DEFAULT_REPLY_DELAY_TIME = Default_Replay_Delay_Time;
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;

	//we set our timer delay
	DEFAULT_TIMER_DELAY = Default_Timer_Delay;
	_timerDelay       = DEFAULT_TIMER_DELAY;
	
	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}

void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[], const byte channel) {
	// general configuration
	_channel = channel;
	DW1000.newConfiguration();
	DW1000.setDefaults(_channel);
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.setChannel(channel);
	DW1000.enableMode(mode);
	DW1000.useExtendedFrameLength(false); 
	DW1000.enableDebounceClock();
	DW1000.setGPIOMode(LEDRXOK, LED_MODE); // RXOKLED-Modus für GPIO0
	DW1000.setGPIOMode(LEDSFD,  LED_MODE); // SFDLED-Modus für GPIO1
	DW1000.setGPIOMode(LEDRX,   LED_MODE); // RXLED-Modus für GPIO2
	DW1000.setGPIOMode(LEDTX,   LED_MODE); // TXLED-Modus für GPIO3
	DW1000.enableLedBlinking();
	DW1000.useSmartPower(false); // Enable Smart Power for higher performance 
	DW1000.setFrameFilter(false);
	DW1000.setFrameFilterAllowData(false);
	DW1000.commitConfiguration();
}
 
void DW1000RangingClass::generalStart() {
	// attach callback for (successfully) sent and received messages
	buffers.init();
	xTaskCreate(uwbProcessingTask, "UWB_PROC", 4*1024, nullptr, 2, &uwbProcessingTaskHandle);
		
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);

	// anchor starts in receiving mode, awaiting a ranging poll message
	
	if(DEBUG) {
		// DEBUG monitoring
		Serial.print("\nDW1000-ESP32");
		// initialize the driver
		
		Serial.print("\nconfiguration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("\nDevice ID: ");
		Serial.print(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("\nUnique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print("\n short: ");
		Serial.print(string);
		
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("\nNetwork ID & Device Address: ");
		Serial.print(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("\nDevice mode: ");
		Serial.print(msg);
	}
	
	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
}

//Anchor
void DW1000RangingClass::startAsAnchor(char address[], const byte mode[], const byte channel) {
	//defined type as anchor
	_type = ANCHOR;
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("\ndevice address: ");
	Serial.print(address);
	
	// we use first two bytes in addess for short address
	_currentShortAddress[0] = _currentAddress[0];
	_currentShortAddress[1] = _currentAddress[1];
	
	
	//configure the network for mac filtering
	//(device Address, network ID, frequency)
	
	uint16_t currShortAddr = _currentShortAddress[0]*256+_currentShortAddress[1];
	DW1000Ranging.configureNetwork(currShortAddr, 0xDECA, mode, channel);
	//general start:
	generalStart();

	Serial.print("\nANCHOR short address: ");
	Serial.print(currShortAddr, HEX);
}

//Tag
void DW1000RangingClass::startAsTag(char address[], const byte mode[], const byte channel) {
	_type = TAG;
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("\ndevice address: ");
	Serial.print(address);

	
	// we use first two bytes in addess for short address
	_currentShortAddress[0] = _currentAddress[0];
	_currentShortAddress[1] = _currentAddress[1];
	
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)

	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode, channel);
	
	generalStart();
	//defined type as tag
	
	lastSyncTime = (esp_timer_get_time()/MICROS_TO_MILLIS);

	Serial.print("\n### TAG ###");
	char buf[128];
	DW1000.getPrintableDeviceMode(buf);
	Serial.print(String("\n") + buf);
}

// Tag & Anchor
boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device) {
    if (_networkDevicesNumber >= MAX_DEVICES) 
		return false;
	
	// Check if the device already exists in the network
    for (uint8_t i = 0; i < _networkDevicesNumber; ++i) {
        if (_networkDevices[i].isAddressEqual(device) || _networkDevices[i].isShortAddressEqual(device)) {
            // The device already exists, do not add it again
            return false;
        }
    }

    // Add the new device to the network
    memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device)); // Directly copy the device
    _networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);       // Set the index for the new device
    _networkDevicesNumber++;
	if(DEBUG){
		Serial.print("\nDevice count: " + String(_networkDevicesNumber));
	}
    return true;
}

// Tag & Anchor
void DW1000RangingClass::removeNetworkDevices(int16_t index) {
    if (index < 0 || index >= _networkDevicesNumber)
        return;

    for (int16_t i = index; i < _networkDevicesNumber - 1; ++i) {
        _networkDevices[i] = _networkDevices[i + 1]; 
        _networkDevices[i].setIndex(i);
    }
    _networkDevicesNumber--;
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
// Tag & Anchor
void DW1000RangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

// Tag & Anchor
void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

// Tag & Anchor
DW1000Device* DW1000RangingClass::searchDistantDevice(const byte shortAddress[]) {
	//we compare the 2 bytes address with the others
	for(uint8_t i = 0; i < _networkDevicesNumber; ++i) {
		if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0) {
			//we have found our device !
			return &_networkDevices[i];
		}
	}
	return nullptr;
}

DW1000Device* DW1000RangingClass::getDistantDevice() {
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
	return &_networkDevices[_lastDistantDevice];
}
/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

 // Tag & Anchor
void DW1000RangingClass::checkForReset() {
    if (buffers.isRxEmpty() && buffers.isTxEmpty()) {
        if ((esp_timer_get_time()/MICROS_TO_MILLIS) - _lastActivity > _resetPeriod) {
            Serial.print("\nreset Inactive");
			resetInactive();
        }
    }
}


// Tag & Anchor
void DW1000RangingClass::checkForInactiveDevices() {
    for (int16_t i = _networkDevicesNumber - 1; i >= 0; --i) {
        if (_networkDevices[i].isInactive()) {
            // Call the handler for inactive devices if set
            if (_handleInactiveDevice) {
                (*_handleInactiveDevice)(&_networkDevices[i]);
            }
            // Remove the inactive device from the array
            removeNetworkDevices(i);  
            if (DEBUG){
                Serial.print("\nInactive device removed. Remaining devices: " + String(_networkDevicesNumber));
            }
        }
    }
}

// Tag & Anchor
MessageType DW1000RangingClass::detectMessageType(const byte datas[]) {
    switch (datas[0]) {
        case FC_1_BLINK:
            return BLINK;
        case FC_1_SYNC:
            return SYNC;
        case FC_1:
            if (datas[1] == FC_2) {
                return RANGING_INIT; // Long MAC frame message (ranging init)
            }
            if (datas[1] == FC_2_SHORT) {
                // Short MAC frame message (poll, range, range report, etc.)
                switch (datas[SHORT_MAC_LEN]) {
                    case 0:  return POLL;
                    case 1:  return POLL_ACK;
                    case 2:  return RANGE;
                    case 3:  return RANGE_REPORT;
                    case 255: return RANGE_FAILED;
                    default: return ERROR;
                }
            }
            break;
    }
    return ERROR;
}

// Tag & Anchor
void DW1000RangingClass::loop(bool uwbSlot) {
    _uwbSlot = uwbSlot;
    checkForReset();
	handlePeriodicTasks(uwbSlot);
}

void DW1000RangingClass::handleSentAck() {
    frame currentFrame;
    if (!buffers.isTxEmpty()){
		buffers.popTx(currentFrame);
        MessageType lastSendMessageType = currentFrame.messageType;
        if (lastSendMessageType != POLL_ACK && lastSendMessageType != POLL && lastSendMessageType != RANGE) {
            if (DEBUG) {
				Serial.print("\nhandleSentAck: Ignore message type: " + String(lastSendMessageType));
			}
			return;
        }
        switch (_type) {
            case ANCHOR:
                handleSentAckAnchor(lastSendMessageType, currentFrame.data, currentFrame.timestamp);
                break;
            case TAG:
                handleSentAckTag(lastSendMessageType, currentFrame.data, currentFrame.timestamp);
                break;
            default:
                if (DEBUG) {
                    Serial.print("\nhandleSentAck: Unknown device type.");
                }
                break;
        }
    }
}

// Tag & Anchor
void DW1000RangingClass::handlePeriodicTasks(bool uwbSlot) {
    // Check if timer has expired
    uint32_t currentTime = (esp_timer_get_time() / MICROS_TO_MILLIS);
	if (currentTime - timer < _timerDelay) {
		return; // Not enough time has passed yet
    }

    // Tasks for TAGs
	if (counterForBlink == 0) {
		if (_type == TAG) {
			transmitBlink(); // Sending Blink signal
		}
		checkForInactiveDevices(); // Check for inactive devices (TAGs and ANCHORs)
	} else if ( _networkDevicesNumber > 0 && _type == TAG){
		transmitPoll(uwbSlot);
	}

	// Updating counter
    counterForBlink = (counterForBlink + 1) % MAX_BLINK_COUNTER;
    timer = currentTime;
}

//Anchor
void DW1000RangingClass::handleSentAckAnchor(MessageType messageType, const uint8_t* data, DW1000Time txTimestamp) {
	if (messageType != POLL_ACK) {
        return; // Exit early if the message type is not relevant
    }
	byte shortAddress[2];
	_globalMac.decodeDestination((byte*)data, shortAddress);
    DW1000Device* myDistantDevice = searchDistantDevice(shortAddress);
    if (myDistantDevice) {
		myDistantDevice->timePollAckSent = txTimestamp;
		myDistantDevice->noteActivity();
    }
}

// Tag
void DW1000RangingClass::handleSentAckTag(MessageType messageType, const uint8_t* data, DW1000Time txTimestamp) {
  	if (messageType != POLL && messageType != RANGE) {
        return; 
	}
	// Retrieve the transmit timestamp for the specific message type (Poll, Range)
	byte shortAddress[2];
	_globalMac.decodeDestination((byte*)data, shortAddress);
	updateDeviceTimeStamps(shortAddress, txTimestamp, messageType);
}

// Tag
void DW1000RangingClass::updateDeviceTimeStamps(byte* shortAddress, DW1000Time time, MessageType messageType) {
	if (shortAddress[0] == 0xFF && shortAddress[1] == 0xFF) {
		for (uint16_t i = 0; i < _networkDevicesNumber; ++i) {
			if (messageType == POLL) {
				_networkDevices[i].timePollSent = time;
				_networkDevices[i].noteActivity();
			} else if (messageType == RANGE) {
				_networkDevices[i].timeRangeSent = time;
				_networkDevices[i].noteActivity();
			}
		}
	} else {
		DW1000Device* myDistantDevice = searchDistantDevice(shortAddress);
		if (myDistantDevice) {
			if (messageType == POLL) {
				myDistantDevice->timePollSent = time;
				myDistantDevice->noteActivity();
			} else if (messageType == RANGE) {
				myDistantDevice->timeRangeSent = time;
				myDistantDevice->noteActivity();
			}
		}
	}
}

// Tag & Anchor 
void DW1000RangingClass::uwbProcessingTask(void *pvParameters){
	uint32_t notificationValue;
    for (;;) {
        // Warte auf RX- oder TX-Event
        xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);

        // RX-Event
        if (notificationValue & 0x01) {
            handleReceivedMessage();
			noteActivity();
        }
        // TX-Event
        if (notificationValue & 0x02) {
            handleSentAck();
			noteActivity();
        }
    }
}

void DW1000RangingClass::handleRxEvent(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Set bit 0 for RX
    xTaskNotifyFromISR(uwbProcessingTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}

void DW1000RangingClass::handleTxEvent(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Set bit 1 for TX
    xTaskNotifyFromISR(uwbProcessingTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}

void DW1000RangingClass::handleReceivedMessage() {
	frame currentFrame;
    if(!buffers.isRxEmpty()) {   
		buffers.popRx(currentFrame);
		MessageType messageType = currentFrame.messageType;
		switch (messageType) {
			case BLINK:
				if (DEBUG) {
					Serial.print("\n[DEBUG] handleReceivedMessage: BLINK received!");
				}
				if (_type == ANCHOR)
					handleBlink(currentFrame.data);
				break;

			case RANGING_INIT:
				if (DEBUG) {
					Serial.print("\n[DEBUG] handleReceivedMessage: RANGING_INIT received!");
				}
				if (_type == TAG)
					handleRangingInit(currentFrame.data);
				break;

			default:
				if (DEBUG) {
					Serial.print("\n[DEBUG] handleReceivedMessage: ");
					Serial.print(messageType);
					Serial.print(" received!");
				}
				processShortMacMessage(messageType, currentFrame);
				break;
		}
	}
}


// Anchor
void DW1000RangingClass::handleBlink(const uint8_t* buffer) {
	byte address[8];
	byte shortAddress[2];
	const uint8_t* data = buffer;
	_globalMac.decodeBlinkFrame((byte*)data, address, shortAddress);
	if (DEBUG) {
		Serial.print("\ndecodeBlinkFrame: address = ");
		for (int i = 0; i < 8; i++) {
			Serial.print(address[i], HEX);
			Serial.print(i < 7 ? ':' : ' ');
		}
		Serial.print("shortAddress = ");
		for (int i = 0; i < 2; i++) {
			Serial.print(shortAddress[i], HEX);
			Serial.print(i < 1 ? ':' : '\n');
		}
	}

	DW1000Device myTag(address, shortAddress);
	if (addNetworkDevices(&myTag)) {
		if (_handleBlinkDevice != 0) {
			(*_handleBlinkDevice)(&myTag);
		}
		transmitRangingInit(&myTag);
		noteActivity();
		DW1000Device* myDistantDevice = searchDistantDevice(shortAddress);
		if (myDistantDevice){
			myDistantDevice->_expectedMsgId = POLL;
			myDistantDevice->noteActivity();
		}
	}
}

// Tag
void DW1000RangingClass::handleRangingInit(const uint8_t* buffer) {
	byte address[2];
	const uint8_t* data = buffer;
	_globalMac.decodeLongMACFrame((byte*)data, address);
	if (DEBUG) {
		Serial.print("handleRangingInit: address = ");
		for (int i = 0; i < 2; i++) {
			Serial.print(address[i], HEX);
			Serial.print(i < 7 ? ':' : ' ');
		}
	}
	DW1000Device myAnchor(address, true);
	if (addNetworkDevices(&myAnchor)) {
		DW1000Device* realAnchor = searchDistantDevice(address);
		if (realAnchor) {
			if (_handleNewDevice != 0) {
				(*_handleNewDevice)(realAnchor);  
			}
			realAnchor->_expectedMsgId = POLL;    
			realAnchor->noteActivity();         
		}
	}
	noteActivity(); 
}

// Tag & Anchor
void DW1000RangingClass::processShortMacMessage(MessageType messageType, const frame& frame) {
	byte address[2];
	const uint8_t* data = frame.data;
	_globalMac.decodeShortMACFrame((byte*)data, address);
	if (DEBUG) {
		Serial.print("\nprocessShortMacMessage: address = ");
		for (int i = 0; i < 2; i++) {
			Serial.print(address[i], HEX);
			Serial.print(i < 7 ? ':' : ' ');
		}
	}
	DW1000Device* myDistantDevice = searchDistantDevice(address);

	if ((_networkDevicesNumber == 0) || (myDistantDevice == nullptr)) {
		if (DEBUG) {
			Serial.print("\nNot found");
		} 
		return;
	}
	myDistantDevice->noteActivity();
	if (_type == ANCHOR) {
		processAnchorMessage(messageType, myDistantDevice, frame);
	} else if (_type == TAG) {
		processTagMessage(messageType, myDistantDevice, frame);
	}
}

// Anchor
void DW1000RangingClass::processAnchorMessage(MessageType messageType, DW1000Device* myDistantDevice,const frame& frame ) {
	if (messageType != myDistantDevice->_expectedMsgId) {
		if (DEBUG) {
			Serial.print("\nNot my _expectedMsgId: ");
			Serial.print(myDistantDevice->_expectedMsgId);
		}
		transmitRangeFailed(myDistantDevice);
		myDistantDevice->_expectedMsgId = POLL;
		_protocolFailed = true;
		return;
	}

	if (messageType == POLL) {
		handlePoll(myDistantDevice, frame);
	} else if (messageType == RANGE) {
		handleRange(myDistantDevice, frame);
	}
}

//anchor
void DW1000RangingClass::handlePoll(DW1000Device* myDistantDevice, const frame& frame) {
    if (!myDistantDevice) {
        if(DEBUG){
            Serial.print("\nhandlePoll: Invalid device pointer");
        }
        return;
    }

	const uint8_t numberDevices = frame.data[SHORT_MAC_LEN + 1];
    const uint8_t* deviceData = frame.data + SHORT_MAC_LEN + 2;

	if(DEBUG){
		Serial.print("\nhandlePoll - numberDevices: ");
		Serial.print(numberDevices);
		Serial.print(", deviceData = ");
		for (int i = 0; i < numberDevices * 4; ++i) { // Display for all devices (4 bytes per device)
			Serial.print(deviceData[i], HEX);
			Serial.print(" ");
		}
	}

	for (uint8_t i = 0; i < numberDevices; i++) {
		const uint8_t* currentDevice = deviceData + i * 4;
		if (memcmp(currentDevice, _currentShortAddress, 2) == 0) {
			_replyDelayTimeUS = *reinterpret_cast<const uint16_t*>(currentDevice + 2);
			_protocolFailed = false;
			myDistantDevice->timePollReceived = frame.timestamp;
			myDistantDevice->_expectedMsgId = RANGE;
			myDistantDevice->noteActivity();
			transmitPollAck(myDistantDevice);
			noteActivity();
			return;
		}
	}

	if(DEBUG){
        Serial.print("\nhandlePoll: Device not found in the poll message");
    }
}


// Anchor
void DW1000RangingClass::handleRange(DW1000Device* myDistantDevice, const frame& frame) {
	uint8_t numberDevices = 0;
	memcpy(&numberDevices, frame.data + SHORT_MAC_LEN + 1, 1);

	for (uint8_t i = 0; i < numberDevices; i++) {
		byte shortAddress[2];
		memcpy(shortAddress, frame.data + SHORT_MAC_LEN + 2 + i * 17, 2);

		if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
			myDistantDevice->timeRangeReceived = frame.timestamp;
		    noteActivity();
			myDistantDevice->_expectedMsgId = POLL;
			myDistantDevice->noteActivity();
			if (!_protocolFailed) {
				myDistantDevice->timePollSent.setTimestamp((byte*)frame.data + SHORT_MAC_LEN + 4 + 17 * i);
				myDistantDevice->timePollAckReceived.setTimestamp((byte*)frame.data + SHORT_MAC_LEN + 9 + 17 * i);
				myDistantDevice->timeRangeSent.setTimestamp((byte*)frame.data + SHORT_MAC_LEN + 14 + 17 * i);
				DW1000Time myTOF;
				computeRangeAsymmetric(myDistantDevice, &myTOF);
				float distance = myTOF.getAsMeters();
				if (_useRangeFilter) {
					if (myDistantDevice->getRange() != 0.0f) {
						distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
					}
				}
				myDistantDevice->setRXPower(frame.receivePower);
				myDistantDevice->setRange(distance);
				myDistantDevice->setFPPower(frame.firstPathPower);
				myDistantDevice->setQuality(frame.receiveQuality);

				transmitRangeReport(myDistantDevice);
				uint32_t currentTime = esp_timer_get_time()/MICROS_TO_MILLIS;
				Serial.print("\nNew Range - round trip time: " + String(currentTime - roundTripTime));
				roundTripTime = currentTime;
				_lastDistantDevice = myDistantDevice->getIndex();
				if (_handleNewRange != 0) {
					(*_handleNewRange)();
				}
			} else {
				transmitRangeFailed(myDistantDevice);
			}
			return;
		}
	}
}

// Tag 
void DW1000RangingClass::processTagMessage(MessageType messageType, DW1000Device* myDistantDevice, const frame& frame ) {
    if (!myDistantDevice) {
        if(DEBUG){
            Serial.print("\nprocessTagMessage: Invalid device pointer");
        }
        return;
    }

    if (messageType != myDistantDevice->_expectedMsgId) {
        if(DEBUG){
            Serial.printf("\nUnexpected message type. Expected: %d, Received: %d\n", myDistantDevice->_expectedMsgId, messageType);
        }
        myDistantDevice->_expectedMsgId = POLL_ACK;
        return;
    }

    switch (messageType) {
        case POLL_ACK:
            handlePollAck(myDistantDevice, frame);
            break;
        case RANGE_REPORT:
            handleRangeReport(myDistantDevice, frame);
            break;
        case RANGE_FAILED:
            if(DEBUG){
                Serial.print("\nRange measurement failed");
            }
            break;
        default:
            if(DEBUG){
                Serial.printf("\nUnhandled message type: %d\n", messageType);
            }
            break;
    }
}

// Tag
void DW1000RangingClass::handlePollAck(DW1000Device* myDistantDevice, const frame& frame) {
	myDistantDevice->timePollAckReceived = frame.timestamp;
    if(myDistantDevice->getIndex() == _networkDevicesNumber-1) {
		myDistantDevice->_expectedMsgId = RANGE_REPORT;
		myDistantDevice->noteActivity();
		transmitRange();
	}
}

// Tag
void DW1000RangingClass::handleRangeReport(DW1000Device* myDistantDevice, const frame& frame) {
    float curRange, curRXPower;
    memcpy(&curRange, frame.data + 1 + SHORT_MAC_LEN, sizeof(float));
    memcpy(&curRXPower, frame.data + 5 + SHORT_MAC_LEN, sizeof(float));

    if (_useRangeFilter && myDistantDevice->getRange() != 0.0f) {
        curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
    }
	myDistantDevice->noteActivity();
    myDistantDevice->setRange(curRange);
    myDistantDevice->setRXPower(curRXPower);
    _lastDistantDevice = myDistantDevice->getIndex();

    if (_handleNewRange) {
        (*_handleNewRange)();
    }

    if (DEBUG) {
        Serial.printf("\nRange: %.2f m, RX Power: %.2f dBm (Device: 0x%02X)\n",
                      curRange, curRXPower, myDistantDevice->getShortAddress());
    }
}

void DW1000RangingClass::useRangeFilter(boolean enabled) {
	_useRangeFilter = enabled;
}

void DW1000RangingClass::setRangeFilterValue(uint16_t newValue) {
	if (newValue < 2) {
		_rangeFilterValue = 2;
	}else{
		_rangeFilterValue = newValue;
	}
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */

//julian 
void DW1000RangingClass::handleSent() {
	DW1000.getTransmitTimestamp(newTxFrame.timestamp);	
	buffers.pushTxOverwrite(newTxFrame);
}

void DW1000RangingClass::handleReceived() {
	if (DEBUG) {
		Serial.printf("\nhandleReceived");	
	}
	frame newFrame;
	newFrame.len = DW1000.getDataLength();
	DW1000.getReceiveTimestamp(newFrame.timestamp);
	DW1000.getData(newFrame.data, newFrame.len);
	newFrame.messageType = detectMessageType(newFrame.data);
	if(newFrame.messageType == RANGE || newFrame.messageType == RANGE_REPORT){
		newFrame.receiveQuality = DW1000.getReceiveQuality();
		newFrame.firstPathPower = DW1000.getFirstPathPower();
		newFrame.receivePower = DW1000.getReceivePower();
	}
	buffers.pushRxOverwrite(newFrame);	
}

void DW1000RangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = (esp_timer_get_time()/MICROS_TO_MILLIS);
}

void DW1000RangingClass::resetInactive() {
	//if inactive
	if(_type == ANCHOR) {
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			_networkDevices[i]._expectedMsgId = POLL;
		}
		receiver();
	}
	noteActivity();
}

void DW1000RangingClass::copyShortAddress(byte address1[], byte address2[]) {
	*address1     = *address2;
	*(address1+1) = *(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

// Tag & Anchor
void DW1000RangingClass::transmitInit() {
	noteActivity();
	DW1000.newTransmit();
}


// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[], uint16_t len) {
	newTxFrame = {};
	memcpy(newTxFrame.data, datas, len);
    newTxFrame.len = len;
	newTxFrame.messageType = detectMessageType(newTxFrame.data);
	DW1000.setData(datas, len);
	DW1000.startTransmit();
}

// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[], uint16_t len, DW1000Time time) {
	newTxFrame = {};
	memcpy(newTxFrame.data, datas, len);
    newTxFrame.len = len;
	newTxFrame.messageType = detectMessageType(newTxFrame.data);
	DW1000.setDelay(time);
	DW1000.setData(datas, len);
	DW1000.startTransmit();
}

//Tag
void DW1000RangingClass::transmitBlink() {
	if (DEBUG) {
		Serial.print("\n[DEBUG] transmitBlink: ");
	}
	transmitInit();
	byte blinkData[12];
	_globalMac.generateBlinkFrame(blinkData, _currentAddress, _currentShortAddress);
	transmit(blinkData, 12);
}

//Anchor
void DW1000RangingClass::transmitRangingInit(DW1000Device* myDistantDevice) {
	if (DEBUG) {
		Serial.print("\n[DEBUG] transmitRangingInit: ");
	}
	byte rangingInitData[LONG_MAC_LEN+1];
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(rangingInitData, _currentShortAddress, myDistantDevice->getByteAddress());
	rangingInitData[LONG_MAC_LEN] = RANGING_INIT; //we define the function code
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(rangingInitData, LONG_MAC_LEN+1);
}

//Tag - broadcast
void DW1000RangingClass::transmitPoll(bool uwbSlot) {
    if (DEBUG) {
		Serial.print("\n[DEBUG] transmitPoll: ");
	}
	transmitInit();
    // Baue Liste der im Slot gewollten Anchors
    DW1000Device* slotDevices[4]; // maximal 4 pro Slot
    uint8_t slotDeviceCount = 0;

    for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
        uint16_t addr = _networkDevices[i].getShortAddress();
        if( ( uwbSlot && isFirstBlock(addr) ) ||
            (!uwbSlot && isSecondBlock(addr) ) )
        {
            if(slotDeviceCount < 4){
                slotDevices[slotDeviceCount++] = &_networkDevices[i];
				_networkDevices[i]._expectedMsgId = POLL_ACK;
			}
        }
    }

    // Setze Timer-Delays abhängig von aktiver Slot-Größe
    _timerDelay = DEFAULT_TIMER_DELAY + (uint16_t)(slotDeviceCount*3*DEFAULT_REPLY_DELAY_TIME/MICROS_TO_MILLIS);
	byte data[SHORT_MAC_LEN+2+kPollDeviceSize*slotDeviceCount];
    byte shortBroadcast[2] = {0xFF, 0xFF};
    _globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
    data[SHORT_MAC_LEN]   = POLL;
    data[SHORT_MAC_LEN+1] = slotDeviceCount;

    // Füge nur die slotgewählten Devices an!
    for(uint8_t i = 0; i < slotDeviceCount; i++) {
        // Reply-Delays für jedes Slot-Device setzen
        slotDevices[i]->setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
        memcpy(data + SHORT_MAC_LEN+2 + kPollDeviceSize*i, slotDevices[i]->getByteShortAddress(), 2);
        uint16_t replyTime = slotDevices[i]->getReplyTime();
        memcpy(data + SHORT_MAC_LEN+2 + 2 + kPollDeviceSize*i, &replyTime, 2);
    }
	for(int i=0; i<slotDeviceCount; i++) {
		_lastSlotDevices[i] = slotDevices[i];
		_slotPollAcksReceived[i] = false;
	}
	_lastSlotDeviceCount = slotDeviceCount;

    copyShortAddress(_lastSentToShortAddress, shortBroadcast);
    transmit(data, SHORT_MAC_LEN+2+kPollDeviceSize*slotDeviceCount);
}


//Anchor
void DW1000RangingClass::transmitPollAck(DW1000Device* myDistantDevice) {
	transmitInit();
	byte pollAckData[SHORT_MAC_LEN+1];
	_globalMac.generateShortMACFrame(pollAckData, _currentShortAddress, myDistantDevice->getByteShortAddress());
	pollAckData[SHORT_MAC_LEN] = POLL_ACK;
	if (DEBUG) {
		Serial.print("\n[DEBUG] transmitPollAck: ");
		Serial.print("\n[PollAck DATA] data[] = ");
		for (int i = 0; i < SHORT_MAC_LEN+1; i++) {
			if (i > 0) Serial.print(":");
			Serial.printf("%02X", pollAckData[i]);
		}
		Serial.print(" (LEN = ");
		Serial.print(SHORT_MAC_LEN+1);
		Serial.println(" Bytes)");
	}

	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(pollAckData, SHORT_MAC_LEN+1, deltaTime);
}

//Tag
void DW1000RangingClass::transmitRange() {
    if (DEBUG) {
		Serial.print("\n[DEBUG] transmitRange: ");
	}
	transmitInit();
    uint8_t slotDeviceCount = _lastSlotDeviceCount;
    _timerDelay = DEFAULT_TIMER_DELAY + (uint16_t)(slotDeviceCount * 3 * DEFAULT_REPLY_DELAY_TIME / MICROS_TO_MILLIS);
	byte rangeData[SHORT_MAC_LEN + 2 + kRangeDeviceSize * slotDeviceCount];
    byte shortBroadcast[2] = {0xFF, 0xFF};
    _globalMac.generateShortMACFrame(rangeData, _currentShortAddress, shortBroadcast);
    rangeData[SHORT_MAC_LEN] = RANGE;
    rangeData[SHORT_MAC_LEN + 1] = slotDeviceCount;
    DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
    DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
    for(uint8_t i = 0; i < slotDeviceCount; i++) {
        DW1000Device* dev = _lastSlotDevices[i];
        memcpy(rangeData + SHORT_MAC_LEN + 2 + kRangeDeviceSize * i, dev->getByteShortAddress(), 2);
        dev->timeRangeSent = timeRangeSent;
        dev->timePollSent.getTimestamp(rangeData + SHORT_MAC_LEN + 4 + kRangeDeviceSize * i);
        dev->timePollAckReceived.getTimestamp(rangeData + SHORT_MAC_LEN + 9 + kRangeDeviceSize * i);
        dev->timeRangeSent.getTimestamp(rangeData + SHORT_MAC_LEN + 14 + kRangeDeviceSize * i);
    }
    copyShortAddress(_lastSentToShortAddress, shortBroadcast);
    transmit(rangeData, SHORT_MAC_LEN + 2 + kRangeDeviceSize * slotDeviceCount);
}


// Anchor
void DW1000RangingClass::transmitRangeReport(DW1000Device* myDistantDevice) {
	if (DEBUG) {
		Serial.print("\n[DEBUG] transmitRangeReport: ");
	}
	transmitInit();
	byte rangeReportData[SHORT_MAC_LEN+9];
	_globalMac.generateShortMACFrame(rangeReportData, _currentShortAddress, myDistantDevice->getByteShortAddress());
	rangeReportData[SHORT_MAC_LEN] = RANGE_REPORT;
	// write final ranging result
	float curRange   = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	
	//We add the Range and then the RXPower
	memcpy(rangeReportData+1+SHORT_MAC_LEN, &curRange, 4);
	memcpy(rangeReportData+5+SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(rangeReportData, SHORT_MAC_LEN+9, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS)); 
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device* myDistantDevice) {
	if (DEBUG) {
		Serial.print("\n[DEBUG] transmitRangeFailed: ");
	}
	transmitInit();
	byte rangeFailedData[SHORT_MAC_LEN+1];
	_globalMac.generateShortMACFrame(rangeFailedData, _currentShortAddress, myDistantDevice->getByteShortAddress());
	rangeFailedData[SHORT_MAC_LEN] = RANGE_FAILED;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(rangeFailedData, SHORT_MAC_LEN+1);
}

void DW1000RangingClass::receiver() {
	DW1000.newReceive();
	DW1000.setDefaults(_channel);
	// so we don't need to restart the receiver manually
	DW1000.alignDoubleBufferPointers();
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */

void DW1000RangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF) {
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
	
	myTOF->setTimestamp((round1*round2-reply1*reply2)/(round1+round2+reply1+reply2));
	if(DEBUG){
		Serial.print("\ntimePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("\ntimePollSent ");myDistantDevice->timePollSent.print();
		Serial.print("\nround1 "); Serial.print((long)round1.getTimestamp());
		
		Serial.print("\ntimePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("\ntimePollReceived ");myDistantDevice->timePollReceived.print();
		Serial.print("\nreply1 "); Serial.print((long)reply1.getTimestamp());
		
		Serial.print("\ntimeRangeReceived ");myDistantDevice->timeRangeReceived.print();
		Serial.print("\ntimePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("\nround2 "); Serial.print((long)round2.getTimestamp());
		
		Serial.print("\ntimeRangeSent ");myDistantDevice->timeRangeSent.print();
		Serial.print("\ntimePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("\nreply2 "); Serial.print((long)reply2.getTimestamp());
	}
}


/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[]) {
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8], datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15]);
	Serial.print(string);
}



/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000RangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements) {
	
	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}

/* ###########################################################################
 * #### New  ###############################################################
 * ######################################################################### */

bool DW1000RangingClass::isFirstBlock(uint16_t shortAddr) {
    // Slot 1: 0x81-0x84, Slot 2: 0x85-0x88
    return shortAddr >= 0x81 && shortAddr <= 0x84;
}

bool DW1000RangingClass::isSecondBlock(uint16_t shortAddr) {
    return shortAddr >= 0x85 && shortAddr <= 0x88;
}