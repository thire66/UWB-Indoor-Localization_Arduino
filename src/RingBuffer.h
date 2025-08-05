#ifndef ESP32_RINGBUFFER_H
#define ESP32_RINGBUFFER_H

#include <cstdint>
#include <stdexcept>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "DW1000Time.h"

// Frame-Struktur mit korrekten Includes
struct frame {
    DW1000Time timestamp;
    uint16_t len;
    uint8_t data[256];
    float receiveQuality;
    float firstPathPower;
    float receivePower;
};

template<typename T, size_t QUEUE_SIZE>
class ESP32RingBuffer {
private:
    T* queue;
    size_t head;
    size_t tail;
    size_t count;
    SemaphoreHandle_t mutex;
    bool dynamicAllocation;
    
public:
    // Konstruktor mit intelligenter Speicherallokation
    ESP32RingBuffer() : head(0), tail(0), count(0), dynamicAllocation(true) {
        // Versuche PSRAM zuerst, dann normales RAM
        #ifdef CONFIG_SPIRAM_SUPPORT
        queue = (T*)heap_caps_malloc(sizeof(T) * QUEUE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (queue) {
            Serial.println("[RingBuffer] Using PSRAM");
        }
        #endif
        
        if (!queue) {
            queue = (T*)heap_caps_malloc(sizeof(T) * QUEUE_SIZE, MALLOC_CAP_DMA);
            if (queue) {
                Serial.println("[RingBuffer] Using DMA-capable RAM");
            }
        }
        
        if (!queue) {
            queue = (T*)malloc(sizeof(T) * QUEUE_SIZE);
            if (queue) {
                Serial.println("[RingBuffer] Using standard heap");
            }
        }
        
        if (!queue) {
            Serial.printf("[RingBuffer] ERROR: Failed to allocate %d bytes\n", 
                         sizeof(T) * QUEUE_SIZE);
            dynamicAllocation = false;
        }
        
        mutex = xSemaphoreCreateMutex();
        if (!mutex) {
            Serial.println("[RingBuffer] ERROR: Failed to create mutex");
        }
        
        Serial.printf("[RingBuffer] Initialized: %d frames, %d bytes each, total: %d bytes\n", 
                     QUEUE_SIZE, sizeof(T), sizeof(T) * QUEUE_SIZE);
    }
    
    // Destruktor
    ~ESP32RingBuffer() {
        if (queue && dynamicAllocation) {
            free(queue);
        }
        if (mutex) {
            vSemaphoreDelete(mutex);
        }
    }
    
    // Initialisierung prüfen
    bool isValid() const {
        return (queue != nullptr && mutex != nullptr);
    }
    
    // Thread-safe Push mit Timeout
    bool push(const T& item, TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid()) return false;
        
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            bool result = false;
            if (!isFullUnsafe()) {
                queue[tail] = item;
                tail = (tail + 1) % QUEUE_SIZE;
                ++count;
                result = true;
            }
            xSemaphoreGive(mutex);
            return result;
        }
        return false;
    }
    
    // Push mit Überschreibung (für kontinuierliche Datenströme)
    bool pushOverwrite(const T& item, TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid()) return false;
        
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            queue[tail] = item;
            tail = (tail + 1) % QUEUE_SIZE;
            
            if (count < QUEUE_SIZE) {
                ++count;
            } else {
                // Buffer war voll, head muss auch weiterbewegt werden
                head = (head + 1) % QUEUE_SIZE;
            }
            xSemaphoreGive(mutex);
            return true;
        }
        return false;
    }
    
    // Thread-safe Pop mit Timeout
    bool pop(T& item, TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid()) return false;
        
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            bool result = false;
            if (!isEmptyUnsafe()) {
                item = queue[head];
                head = (head + 1) % QUEUE_SIZE;
                --count;
                result = true;
            }
            xSemaphoreGive(mutex);
            return result;
        }
        return false;
    }
    
    // Non-blocking Peek (erstes Element ansehen)
    bool peek(T& item, TickType_t timeout = pdMS_TO_TICKS(10)) const {
        if (!isValid()) return false;
        
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            bool result = false;
            if (!isEmptyUnsafe()) {
                item = queue[head];
                result = true;
            }
            xSemaphoreGive(mutex);
            return result;
        }
        return false;
    }
    
    // Batch-Operationen für effiziente DW1000-Datenverarbeitung
    size_t pushMultiple(const T* items, size_t numItems, TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid() || !items) return 0;
        
        size_t pushed = 0;
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            for (size_t i = 0; i < numItems && !isFullUnsafe(); ++i) {
                queue[tail] = items[i];
                tail = (tail + 1) % QUEUE_SIZE;
                ++count;
                ++pushed;
            }
            xSemaphoreGive(mutex);
        }
        return pushed;
    }
    
    size_t popMultiple(T* items, size_t maxItems, TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid() || !items) return 0;
        
        size_t popped = 0;
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            for (size_t i = 0; i < maxItems && !isEmptyUnsafe(); ++i) {
                items[i] = queue[head];
                head = (head + 1) % QUEUE_SIZE;
                --count;
                ++popped;
            }
            xSemaphoreGive(mutex);
        }
        return popped;
    }
    
    // Status-Funktionen (thread-safe)
    bool isEmpty() const {
        if (!isValid()) return true;
        
        bool result = true;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            result = isEmptyUnsafe();
            xSemaphoreGive(mutex);
        }
        return result;
    }
    
    bool isFull() const {
        if (!isValid()) return false;
        
        bool result = false;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            result = isFullUnsafe();
            xSemaphoreGive(mutex);
        }
        return result;
    }
    
    size_t size() const {
        if (!isValid()) return 0;
        
        size_t result = 0;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            result = count;
            xSemaphoreGive(mutex);
        }
        return result;
    }
    
    size_t capacity() const {
        return QUEUE_SIZE;
    }
    
    size_t available() const {
        return QUEUE_SIZE - size();
    }
    
    // Buffer leeren
    void clear(TickType_t timeout = pdMS_TO_TICKS(100)) {
        if (!isValid()) return;
        
        if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
            head = 0;
            tail = 0;
            count = 0;
            xSemaphoreGive(mutex);
        }
    }
    
    // ESP32-spezifische Debug-Funktionen
    void printStatus() const {
        Serial.printf("\n[RingBuffer] Size: %d/%d, Head: %d, Tail: %d, Free Heap: %d\n", 
                     size(), QUEUE_SIZE, head, tail, ESP.getFreeHeap());
    }
    
    size_t getMemoryUsage() const {
        return sizeof(T) * QUEUE_SIZE;
    }
    
    float utilizationPercent() const {
        return (static_cast<float>(size()) / QUEUE_SIZE) * 100.0f;
    }
    
    // DW1000-spezifische Hilfsfunktionen
    bool getLatestFrame(T& frame) const {
        if (!isValid()) return false;
        
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool result = false;
            if (!isEmptyUnsafe()) {
                // Letztes eingefügtes Element (tail - 1)
                size_t lastIndex = (tail == 0) ? QUEUE_SIZE - 1 : tail - 1;
                frame = queue[lastIndex];
                result = true;
            }
            xSemaphoreGive(mutex);
            return result;
        }
        return false;
    }
    
    // Frames nach Timestamp-Bereich suchen
    size_t getFramesInTimeRange(const DW1000Time& startTime, const DW1000Time& endTime, 
                               T* results, size_t maxResults) const {
        if (!isValid() || !results) return 0;
        
        size_t found = 0;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (size_t i = 0; i < count && found < maxResults; ++i) {
                size_t index = (head + i) % QUEUE_SIZE;
                if (queue[index].timestamp >= startTime && queue[index].timestamp <= endTime) {
                    results[found++] = queue[index];
                }
            }
            xSemaphoreGive(mutex);
        }
        return found;
    }

private:
    // Unsafe Versionen (nur innerhalb von Mutex-geschützten Bereichen verwenden)
    bool isEmptyUnsafe() const {
        return count == 0;
    }
    
    bool isFullUnsafe() const {
        return count == QUEUE_SIZE;
    }
};

// Spezialisierung für DW1000 Frames
using DW1000FrameRingBuffer = ESP32RingBuffer<frame, 16>;

#endif // ESP32_RINGBUFFER_H
