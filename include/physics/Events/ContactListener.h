#pragma once
#include "CollisionEvent.h"

class ContactListener {
public:
    virtual ~ContactListener() = default;

    virtual void OnCollisionEnter(const CollisionEvent& event) {}
    virtual void OnCollisionStay(const CollisionEvent& event) {}
    virtual void OnCollisionExit(const CollisionEvent& event) {}
    virtual void OnTriggerEnter(const TriggerEvent& event) {}
    virtual void OnTriggerStay(const TriggerEvent& event) {}
    virtual void OnTriggerExit(const TriggerEvent& event) {}
};