/*
Gère les ombres pour la scène
*/

#include "Shadow.h"

Shadow::Shadow(VkDevice edevice, VkQueue queue, VkDescriptorBufferInfo* lights, VkFormat depthFormat, VulkanExampleBase* mainClass,VkPipelineVertexInputStateCreateInfo* verticesState):exampleBase(mainClass),device(edevice),fbDepthFormat(depthFormat)
{
    prepareUniformBuffers();
    prepareCubeMap(queue);
    setupDescriptorSetLayout();
    prepareOffscreenRenderpass();
    preparePipeline(verticesState);
    setupDescriptorSets(lights);
    prepareOffscreenFramebuffer(queue);
}

vkTools::VulkanTexture* Shadow::getCubeMapTexture()
{
    return &shadowCubeMap;
}

Shadow::~Shadow()
{
    // Color attachment
    vkDestroyImageView(device, offScreenFrameBuf.color.view, nullptr);
    vkDestroyImage(device, offScreenFrameBuf.color.image, nullptr);
    vkFreeMemory(device, offScreenFrameBuf.color.mem, nullptr);

    // Depth attachment
    vkDestroyImageView(device, offScreenFrameBuf.depth.view, nullptr);
    vkDestroyImage(device, offScreenFrameBuf.depth.image, nullptr);
    vkFreeMemory(device, offScreenFrameBuf.depth.mem, nullptr);

    vkDestroyFramebuffer(device, offScreenFrameBuf.frameBuffer, nullptr);
}

void Shadow::prepareCubeMap(VkQueue queue)
{
    shadowCubeMap.width = TEX_DIM;
    shadowCubeMap.height = TEX_DIM;

    // 32 bit float format for higher precision
    VkFormat format = VK_FORMAT_R32_SFLOAT;

    // Cube map image description
    VkImageCreateInfo imageCreateInfo = vkTools::initializers::imageCreateInfo();
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = format;
    imageCreateInfo.extent = { shadowCubeMap.width, shadowCubeMap.height, 1 };
    imageCreateInfo.mipLevels = 1;
    imageCreateInfo.arrayLayers = 6;
    imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageCreateInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;

    VkMemoryAllocateInfo memAllocInfo = vkTools::initializers::memoryAllocateInfo();
    VkMemoryRequirements memReqs;

    VkCommandBuffer layoutCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY,true);

    // Create cube map image
    VK_CHECK_RESULT(vkCreateImage(device, &imageCreateInfo, nullptr, &shadowCubeMap.image));

    vkGetImageMemoryRequirements(device, shadowCubeMap.image, &memReqs);

    memAllocInfo.allocationSize = memReqs.size;
    memAllocInfo.memoryTypeIndex = exampleBase->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &shadowCubeMap.deviceMemory));
    VK_CHECK_RESULT(vkBindImageMemory(device, shadowCubeMap.image, shadowCubeMap.deviceMemory, 0));

    // Image barrier for optimal image (target)
    VkImageSubresourceRange subresourceRange = {};
    subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    subresourceRange.baseMipLevel = 0;
    subresourceRange.levelCount = 1;
    subresourceRange.layerCount = 6;
    vkTools::setImageLayout(
        layoutCmd,
        shadowCubeMap.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        subresourceRange);

    exampleBase->flushCommandBuffer(layoutCmd, queue, true);

    // Create sampler
    VkSamplerCreateInfo sampler = vkTools::initializers::samplerCreateInfo();
    sampler.magFilter = TEX_FILTER;
    sampler.minFilter = TEX_FILTER;
    sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler.addressModeV = sampler.addressModeU;
    sampler.addressModeW = sampler.addressModeU;
    sampler.mipLodBias = 0.0f;
    sampler.maxAnisotropy = 0;
    sampler.compareOp = VK_COMPARE_OP_NEVER;
    sampler.minLod = 0.0f;
    sampler.maxLod = 1.0f;
    sampler.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    VK_CHECK_RESULT(vkCreateSampler(device, &sampler, nullptr, &shadowCubeMap.sampler));

    // Create image view
    VkImageViewCreateInfo view = vkTools::initializers::imageViewCreateInfo();
    view.image = VK_NULL_HANDLE;
    view.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
    view.format = format;
    view.components = { VK_COMPONENT_SWIZZLE_R };
    view.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
    view.subresourceRange.layerCount = 6;
    view.image = shadowCubeMap.image;
    VK_CHECK_RESULT(vkCreateImageView(device, &view, nullptr, &shadowCubeMap.view));
}

// Prepare a new framebuffer for offscreen rendering
// The contents of this framebuffer are then
// copied to the different cube map faces
void Shadow::prepareOffscreenFramebuffer(VkQueue queue)
{
    offScreenFrameBuf.width = FB_DIM;
    offScreenFrameBuf.height = FB_DIM;

    VkFormat fbColorFormat = FB_COLOR_FORMAT;

    // Color attachment
    VkImageCreateInfo imageCreateInfo = vkTools::initializers::imageCreateInfo();
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = fbColorFormat;
    imageCreateInfo.extent.width = offScreenFrameBuf.width;
    imageCreateInfo.extent.height = offScreenFrameBuf.height;
    imageCreateInfo.extent.depth = 1;
    imageCreateInfo.mipLevels = 1;
    imageCreateInfo.arrayLayers = 1;
    imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    // Image of the framebuffer is blit source
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageCreateInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkMemoryAllocateInfo memAlloc = vkTools::initializers::memoryAllocateInfo();

    VkImageViewCreateInfo colorImageView = vkTools::initializers::imageViewCreateInfo();
    colorImageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
    colorImageView.format = fbColorFormat;
    colorImageView.flags = 0;
    colorImageView.subresourceRange = {};
    colorImageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colorImageView.subresourceRange.baseMipLevel = 0;
    colorImageView.subresourceRange.levelCount = 1;
    colorImageView.subresourceRange.baseArrayLayer = 0;
    colorImageView.subresourceRange.layerCount = 1;

    VkMemoryRequirements memReqs;

    VK_CHECK_RESULT(vkCreateImage(device, &imageCreateInfo, nullptr, &offScreenFrameBuf.color.image));
    vkGetImageMemoryRequirements(device, offScreenFrameBuf.color.image, &memReqs);
    memAlloc.allocationSize = memReqs.size;
    memAlloc.memoryTypeIndex = exampleBase->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &offScreenFrameBuf.color.mem));
    VK_CHECK_RESULT(vkBindImageMemory(device, offScreenFrameBuf.color.image, offScreenFrameBuf.color.mem, 0));

    VkCommandBuffer layoutCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY,true);

    vkTools::setImageLayout(
        layoutCmd,
        offScreenFrameBuf.color.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    colorImageView.image = offScreenFrameBuf.color.image;
    VK_CHECK_RESULT(vkCreateImageView(device, &colorImageView, nullptr, &offScreenFrameBuf.color.view));

    // Depth stencil attachment
    imageCreateInfo.format = fbDepthFormat;
    imageCreateInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    VkImageViewCreateInfo depthStencilView = vkTools::initializers::imageViewCreateInfo();
    depthStencilView.viewType = VK_IMAGE_VIEW_TYPE_2D;
    depthStencilView.format = fbDepthFormat;
    depthStencilView.flags = 0;
    depthStencilView.subresourceRange = {};
    depthStencilView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
    depthStencilView.subresourceRange.baseMipLevel = 0;
    depthStencilView.subresourceRange.levelCount = 1;
    depthStencilView.subresourceRange.baseArrayLayer = 0;
    depthStencilView.subresourceRange.layerCount = 1;

    VK_CHECK_RESULT(vkCreateImage(device, &imageCreateInfo, nullptr, &offScreenFrameBuf.depth.image));
    vkGetImageMemoryRequirements(device, offScreenFrameBuf.depth.image, &memReqs);
    memAlloc.allocationSize = memReqs.size;
    memAlloc.memoryTypeIndex = exampleBase->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &offScreenFrameBuf.depth.mem));
    VK_CHECK_RESULT(vkBindImageMemory(device, offScreenFrameBuf.depth.image, offScreenFrameBuf.depth.mem, 0));

    vkTools::setImageLayout(
        layoutCmd,
        offScreenFrameBuf.depth.image,
        VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

    exampleBase->flushCommandBuffer(layoutCmd, queue, true);

    depthStencilView.image = offScreenFrameBuf.depth.image;
    VK_CHECK_RESULT(vkCreateImageView(device, &depthStencilView, nullptr, &offScreenFrameBuf.depth.view));

    VkImageView attachments[2];
    attachments[0] = offScreenFrameBuf.color.view;
    attachments[1] = offScreenFrameBuf.depth.view;

    VkFramebufferCreateInfo fbufCreateInfo = vkTools::initializers::framebufferCreateInfo();
    fbufCreateInfo.renderPass = offScreenFrameBuf.renderPass;
    fbufCreateInfo.attachmentCount = 2;
    fbufCreateInfo.pAttachments = attachments;
    fbufCreateInfo.width = offScreenFrameBuf.width;
    fbufCreateInfo.height = offScreenFrameBuf.height;
    fbufCreateInfo.layers = 1;

    VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &offScreenFrameBuf.frameBuffer));
}

// Updates a single cube map face
// Renders the scene with face's view and does
// a copy from framebuffer to cube face
// Uses push constants for quick update of
// view matrix for the current cube map face
void Shadow::updateCubeFace(VkCommandBuffer offScreenCmdBuffer, uint32_t faceIndex, uint32_t light, std::vector<VulkanObject*> objects, VkBuffer* points)
{


    // Update view matrix via push constant
    pCostant.viewMatrix = glm::scale(glm::mat4(),glm::vec3(-1,-1,1));
    pCostant.lightIndex.x=light;

    switch (faceIndex)
    {
    case 0: // POSITIVE_X
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        //pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(180.0f), glm::vec3(1.0f,0.0f, 0.0f));
        break;
    case 1:	// NEGATIVE_X
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        break;
    case 2:	// POSITIVE_Y
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(180.0f), glm::vec3(0.0f,1.0f, 0.0f));
        break;
    case 3:	// NEGATIVE_Y
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(180.0f), glm::vec3(0.0f,1.0f, 0.0f));
        break;
    case 4:	// POSITIVE_Z
        pCostant.viewMatrix = glm::rotate(pCostant.viewMatrix, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        break;
    }

    VkClearValue clearValues[2];
    clearValues[0].color = { { 0.0f,0.0f, 0.0f, 1.0f } };
    clearValues[1].depthStencil = { 1.0f, 0 };

    renderPassBeginInfo = vkTools::initializers::renderPassBeginInfo();
    // Reuse render pass from example pass
    renderPassBeginInfo.renderPass = exampleBase->renderPass;
    renderPassBeginInfo.framebuffer = offScreenFrameBuf.frameBuffer;
    renderPassBeginInfo.renderArea.extent.width = offScreenFrameBuf.width;
    renderPassBeginInfo.renderArea.extent.height = offScreenFrameBuf.height;
    renderPassBeginInfo.clearValueCount = 2;
    renderPassBeginInfo.pClearValues = clearValues;

    // Render scene from cube face's point of view
    vkCmdBeginRenderPass(offScreenCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

    // Update shader push constant block
    // Contains current face view matrix

    //std::cout<< "pcojaefg "<< pCostant.lightIndex << std::endl;
                vkCmdPushConstants(
        offScreenCmdBuffer,
        shadowPipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT,
        0,
        sizeof(PConst),
        &pCostant);

    vkCmdBindPipeline(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline);
    vkCmdBindDescriptorSets(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout, 0, 1, &descSet, 0, NULL);

    for(VulkanObject* object : objects){
        object->draw(offScreenCmdBuffer,points);
    }

    vkCmdEndRenderPass(offScreenCmdBuffer);
    // Make sure color writes to the framebuffer are finished before using it as transfer source
    vkTools::setImageLayout(
        offScreenCmdBuffer,
        offScreenFrameBuf.color.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

    // Copy region for transfer from framebuffer to cube face
    VkImageCopy copyRegion = {};

    copyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copyRegion.srcSubresource.baseArrayLayer = 0;
    copyRegion.srcSubresource.mipLevel = 0;
    copyRegion.srcSubresource.layerCount = 1;
    copyRegion.srcOffset = { 0, 0, 0 };

    copyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copyRegion.dstSubresource.baseArrayLayer = faceIndex;
    copyRegion.dstSubresource.mipLevel = 0;
    copyRegion.dstSubresource.layerCount = 1;
    copyRegion.dstOffset = { 0, 0, 0 };

    copyRegion.extent.width = shadowCubeMap.width;
    copyRegion.extent.height = shadowCubeMap.height;
    copyRegion.extent.depth = 1;

    // Put image copy into command buffer
    vkCmdCopyImage(
        offScreenCmdBuffer,
        offScreenFrameBuf.color.image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        shadowCubeMap.image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1,
        &copyRegion);

    // Transform framebuffer color attachment back
    vkTools::setImageLayout(
        offScreenCmdBuffer,
        offScreenFrameBuf.color.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
}

// Command buffer for rendering and copying all cube map faces
void Shadow::buildOffscreenCommandBuffer(VkCommandBuffer offScreenCmdBuffer, std::vector<VulkanObject*> objects, VkBuffer* points, uint32_t lightIndex)
{
    //offScreenCmdBuffer = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY,false);

    //VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

    //VK_CHECK_RESULT(vkBeginCommandBuffer(offScreenCmdBuffer, &cmdBufInfo));

    VkViewport viewport = vkTools::initializers::viewport((float)offScreenFrameBuf.width, (float)offScreenFrameBuf.height, 0.0f, 1.0f);
    vkCmdSetViewport(offScreenCmdBuffer, 0, 1, &viewport);

    VkRect2D scissor = vkTools::initializers::rect2D(offScreenFrameBuf.width, offScreenFrameBuf.height,	0, 0);
    vkCmdSetScissor(offScreenCmdBuffer, 0, 1, &scissor);

    VkImageSubresourceRange subresourceRange = {};
    subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    subresourceRange.baseMipLevel = 0;
    subresourceRange.levelCount = 1;
    subresourceRange.layerCount = 6;

    // Change image layout for all cubemap faces to transfer destination
    vkTools::setImageLayout(
        offScreenCmdBuffer,
        shadowCubeMap.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        subresourceRange);


    for (uint32_t face = 0; face < 6; ++face)
    {
        updateCubeFace(offScreenCmdBuffer,face,lightIndex,objects,points);
    }


    // Change image layout for all cubemap faces to shader read after they have been copied
    vkTools::setImageLayout(
        offScreenCmdBuffer,
        shadowCubeMap.image,
        VK_IMAGE_ASPECT_COLOR_BIT,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        subresourceRange);

    //VK_CHECK_RESULT(vkEndCommandBuffer(offScreenCmdBuffer));
}

// Set up a separate render pass for the offscreen frame buffer
// This is necessary as the offscreen frame buffer attachments
// use formats different to the ones from the visible frame buffer
// and at least the depth one may not be compatible
void Shadow::prepareOffscreenRenderpass()
{
    VkAttachmentDescription osAttachments[2] = {};

    osAttachments[0].format = FB_COLOR_FORMAT;
    osAttachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    osAttachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    osAttachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    osAttachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    osAttachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    osAttachments[0].initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    osAttachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    // Depth attachment
    osAttachments[1].format = fbDepthFormat;
    osAttachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    osAttachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    osAttachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    osAttachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    osAttachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    osAttachments[1].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    osAttachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorReference = {};
    colorReference.attachment = 0;
    colorReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthReference = {};
    depthReference.attachment = 1;
    depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorReference;
    subpass.pDepthStencilAttachment = &depthReference;

    VkRenderPassCreateInfo renderPassCreateInfo = vkTools::initializers::renderPassCreateInfo();
    renderPassCreateInfo.attachmentCount = 2;
    renderPassCreateInfo.pAttachments = osAttachments;
    renderPassCreateInfo.subpassCount = 1;
    renderPassCreateInfo.pSubpasses = &subpass;

    VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCreateInfo, nullptr, &offScreenFrameBuf.renderPass));
}

// Prepare and initialize uniform buffer containing shader uniforms
void Shadow::prepareUniformBuffers()
{
    uboOffscreenVS.projection = glm::perspective((float)(M_PI / 2.0), 1.0f, 300.0f, zFar);

    uboOffscreenVS.lightPos[0] = glm::vec3(5.0f,10.0f,0.0f);
    uboOffscreenVS.lightPos[1] = glm::vec3(5.0f,10.0f,0.0f);

    // Offscreen vertex shader uniform buffer block
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        sizeof(uboOffscreenVS),
        &uboOffscreenVS,
        &offscreen.buffer,
        &offscreen.memory,
        &offscreen.descriptor);
}

void Shadow::preparePipeline(VkPipelineVertexInputStateCreateInfo* verticesState)
{
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
        vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
            VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
            0,
            VK_FALSE);

    VkPipelineRasterizationStateCreateInfo rasterizationState =
        vkTools::initializers::pipelineRasterizationStateCreateInfo(
            VK_POLYGON_MODE_FILL,
            VK_CULL_MODE_FRONT_BIT,
            VK_FRONT_FACE_CLOCKWISE,
            0);

    VkPipelineColorBlendAttachmentState blendAttachmentState =
        vkTools::initializers::pipelineColorBlendAttachmentState(
            0xf,
            VK_FALSE);

    VkPipelineColorBlendStateCreateInfo colorBlendState =
        vkTools::initializers::pipelineColorBlendStateCreateInfo(
            1,
            &blendAttachmentState);

    VkPipelineDepthStencilStateCreateInfo depthStencilState =
        vkTools::initializers::pipelineDepthStencilStateCreateInfo(
            VK_TRUE,
            VK_TRUE,
            VK_COMPARE_OP_LESS_OR_EQUAL);

    VkPipelineViewportStateCreateInfo viewportState =
        vkTools::initializers::pipelineViewportStateCreateInfo(1, 1, 0);

    VkPipelineMultisampleStateCreateInfo multisampleState =
        vkTools::initializers::pipelineMultisampleStateCreateInfo(
            VK_SAMPLE_COUNT_1_BIT,
            0);

    std::vector<VkDynamicState> dynamicStateEnables = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };
    VkPipelineDynamicStateCreateInfo dynamicState =
        vkTools::initializers::pipelineDynamicStateCreateInfo(
            dynamicStateEnables.data(),
            dynamicStateEnables.size(),
            0);

    VkGraphicsPipelineCreateInfo pipelineCreateInfo =
        vkTools::initializers::pipelineCreateInfo(
            shadowPipelineLayout,
            offScreenFrameBuf.renderPass,
            0);
    // Load shaders
    std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;
    shaderStages[0] = exampleBase->loadShader(exampleBase->getAssetPath() + "shaders/shadow/offscreen.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
    shaderStages[1] = exampleBase->loadShader(exampleBase->getAssetPath() + "shaders/shadow/offscreen.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

    pipelineCreateInfo.pVertexInputState = verticesState;
    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.stageCount = shaderStages.size();
    pipelineCreateInfo.pStages = shaderStages.data();

    VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, exampleBase->pipelineCache, 1, &pipelineCreateInfo, nullptr, &shadowPipeline));
}

void Shadow::setupDescriptorSetLayout()
{
    // Shared pipeline layout
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
    {
        // Binding 0 : Vertex shader uniform buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            VK_SHADER_STAGE_VERTEX_BIT,
            0),
        // Binding 1 : Vertex shader storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_VERTEX_BIT,
            1)
    };

    VkDescriptorSetLayoutCreateInfo descriptorLayout =
        vkTools::initializers::descriptorSetLayoutCreateInfo(
            setLayoutBindings.data(),
            setLayoutBindings.size());

    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout));

    // 3D scene pipeline layout
    VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
        vkTools::initializers::pipelineLayoutCreateInfo(
            &descriptorSetLayout,
            1);

    // Offscreen pipeline layout
    // Push constants for cube map face view matrices
    VkPushConstantRange pushConstantRange =
        vkTools::initializers::pushConstantRange(
            VK_SHADER_STAGE_VERTEX_BIT,
            sizeof(PConst),
            0);

    // Push constant ranges are part of the pipeline layout
    pPipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pPipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

    VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr, &shadowPipelineLayout));
}

void Shadow::setupDescriptorSets(VkDescriptorBufferInfo* lights)
{
    VkDescriptorSetAllocateInfo allocInfo =
        vkTools::initializers::descriptorSetAllocateInfo(
            exampleBase->descriptorPool,
            &descriptorSetLayout,
            1);

    // Offscreen
    VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descSet));

    std::vector<VkWriteDescriptorSet> offScreenWriteDescriptorSets =
    {
        // Binding 0 : Vertex shader uniform buffer
        vkTools::initializers::writeDescriptorSet(
            descSet,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            0,
            &offscreen.descriptor),
        // Binding 1 : Vertex shader uniform buffer
        vkTools::initializers::writeDescriptorSet(
            descSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            1,
            lights)
    };
    vkUpdateDescriptorSets(device, offScreenWriteDescriptorSets.size(), offScreenWriteDescriptorSets.data(), 0, NULL);
}

/*VkCommandBuffer* Shadow::getCommandBuffer(){
    return &offScreenCmdBuffer;
}*/

