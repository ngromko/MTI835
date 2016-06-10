#pragma once

#include "btBulletDynamicsCommon.h"

class VkParticleMotionState : public btMotionState
{
protected:
    VulkanFire* mSceneNode;
    btTransform mInitialPosition;
    int pindex;

public:
    VkParticleMotionState(const btTransform &initialPosition, VulkanFire* node,int index)
    {
        mSceneNode = node;
        mInitialPosition = initialPosition;
        pindex=index;
    }

    virtual ~VkParticleMotionState()
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

        btVector3 ori = worldTrans.getOrigin();
        mSceneNode->updateParticle(ori,pindex);
    }
};
