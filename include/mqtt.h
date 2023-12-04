#ifndef MQTT_H
#define MQTT_H

namespace MQTT {
    enum QoS {
        // At the lowest level, QoS 0 in MQTT offers a best-effort delivery mechanism where the sender does not expect an acknowledgment or guarantee of message delivery. This means that the recipient does not acknowledge receiving the message, and the sender does not store or re-transmit it. QoS 0, commonly called “fire and forget,” functions akin to the underlying TCP protocol, where the message is sent without further follow-up or confirmation.
        AtMostOnce  = 0,

        // In QoS 1 of MQTT, the focus is on ensuring message delivery at least once to the receiver. When a message is published with QoS 1, the sender keeps a copy of the message until it receives a PUBACK packet from the receiver, confirming the successful receipt. If the sender doesn’t receive the PUBACK packet within a reasonable time frame, it re-transmits the message to ensure its delivery. 
        AtLeastOnce = 1,
        
        // QoS 2 offers the highest level of service in MQTT, ensuring that each message is delivered exactly once to the intended recipients. To achieve this, QoS 2 involves a four-part handshake between the sender and receiver.
        ExactlyOnce = 2,
    };

    enum RetainMessage : bool {
        // Will not save last message for all new subscribers
        No = false, 

        // Will save last message for all new subscribers
        Yes = true,
    };
}

#endif