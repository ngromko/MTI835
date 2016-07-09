#pragma once

#include "btBulletDynamicsCommon.h"

class VkObjectMotionState : public btMotionState
{
protected:
    VulkanObject* mSceneNode;
    btTransform mInitialPosition;

public:
    VkObjectMotionState(const btTransform &initialPosition, VulkanObject* node)
    {
        mSceneNode = node;
        mInitialPosition = initialPosition;
    }

    virtual ~VkObjectMotionState()
    {
    }

    virtual void getWorldTransform(btTransform &worldTrans) const
    {
        worldTrans = mInitialPosition;
    }

    virtual void setWorldTransform(const btTransform &worldTrans)
    {
        if(mSceneNode == nullptr)
            return; // silently return before we set a node

        glm::mat4 temp = glm::mat4();

        btVector3 ori = worldTrans.getOrigin();
        temp = glm::translate(temp,glm::vec3(ori.x(),ori.y(),ori.z()));

        btQuaternion qua = worldTrans.getRotation();

        temp = glm::rotate(temp,qua.getAngle(),glm::vec3(qua.getAxis().x(),qua.getAxis().y(),qua.getAxis().z()));
        mSceneNode->updateModel(temp);
    }
};
