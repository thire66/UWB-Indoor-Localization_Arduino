#ifndef UWB_RINGBUFFERS_H
#define UWB_RINGBUFFERS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "DW1000Time.h"
#include "DW1000Ranging.h"

struct frame {
    DW1000Time timestamp;
    uint16_t len;
    uint8_t data[90];
    float receiveQuality;
    float firstPathPower;
    float receivePower;
    MessageType messageType;
};

class UwbFrameBuffers {
public:
    static constexpr size_t BufferSize = 16;

    UwbFrameBuffers() : rxBuffer(nullptr), txBuffer(nullptr) {}

    void init() {
        rxBuffer = xQueueCreate(BufferSize, sizeof(frame));
        txBuffer = xQueueCreate(BufferSize, sizeof(frame));
    }

    // RX Buffer-Operationen
    bool pushRxOverwrite(const frame& f) { return pushOverwrite(rxBuffer, f); }
    bool popRx(frame& f)                 { return pop(rxBuffer, f); }
    bool isRxEmpty() const               { return isEmpty(rxBuffer); }

    // TX Buffer-Operationen
    bool pushTxOverwrite(const frame& f) { return pushOverwrite(txBuffer, f); }
    bool popTx(frame& f)                 { return pop(txBuffer, f); }
    bool isTxEmpty() const               { return isEmpty(txBuffer); }

private:
    QueueHandle_t rxBuffer;
    QueueHandle_t txBuffer;

    // Hilfsfunktionen (intern genutzt)
    static bool pushOverwrite(QueueHandle_t queue, const frame& f) {
        if (!queue) return false;
        BaseType_t res = xQueueSend(queue, &f, 0);
        if (res == pdPASS) return true;
        frame dummy;
        xQueueReceive(queue, &dummy, 0); // Ältestes rauswerfen
        res = xQueueSend(queue, &f, 0);
        return (res == pdPASS);
    }

    static bool pop(QueueHandle_t queue, frame& f) {
        if (!queue) return false;
        return xQueueReceive(queue, &f, 0) == pdPASS;
    }

    static bool isEmpty(QueueHandle_t queue) {
        if (!queue) return true;
        return uxQueueMessagesWaiting(queue) == 0;
    }
};

#endif // UWB_RINGBUFFERS_H
