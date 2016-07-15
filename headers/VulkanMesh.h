/*
* Vulkan Example - Animated gears using multiple uniform buffers
*
* See readme.md for details
*
* Copyright (C) 2015 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <math.h>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkanObject.h"
#include "vkObjectMotionState.h"

class VulkanMesh : public VulkanObject
{
private:
    bool selectable;    
    void prepareRigidBody(std::vector<BurningPoint> &bPoints, glm::mat4 startPos, float mass, uint32_t objectNumber);
public:
    ~VulkanMesh();
    VulkanMesh(VkDevice mdevice, VulkanExampleBase *mexample, VkQueue queue, std::vector<unsigned int> Indices, bool burnable, glm::mat4 atartModel, float mass, uint32_t objectNumber, uint32_t bPointStart, std::vector<BurningPoint> &bPoints);
};

