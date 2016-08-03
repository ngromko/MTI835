#include "particlefire.h"

VulkanFire::VulkanFire(VkDevice device, VulkanExampleBase *example)
{
    this->device = device;
    this->exampleBase=example; 
}

VulkanFire::~VulkanFire()
{
    // Clean up used Vulkan resources
    // Note : Inherited destructor cleans up resources stored in base class

    vkUnmapMemory(device, computeUniformBuffer.memory);
    vkDestroyBuffer(device, computeUniformBuffer.buffer, nullptr);
    vkFreeMemory(device, computeUniformBuffer.memory, nullptr);

    vkDestroyPipeline(device,drawParticles,nullptr);
    vkDestroyPipeline(device,clickFire,nullptr);
    vkDestroyPipeline(device,updateParticles,nullptr);
    vkDestroyPipeline(device,propageFire,nullptr);
}

void VulkanFire::compute(VkCommandBuffer cmdbuffer)
{
    int bGroups  = 1;
    while(bGroups*512<computeUbo.bPointsCount){
        bGroups++;
    }
    int pGroups  = 1;
    while(pGroups*512<computeUbo.particleCount){
        pGroups++;
    }
/*
    // Add memory barrier to ensure that the (rendering) vertex shader operations have finished
    // Required as the compute shader will overwrite the vertex buffer data
    VkBufferMemoryBarrier bufferBarrier = vkTools::initializers::bufferMemoryBarrier();
    // Vertex shader invocations have finished reading from the buffer
    bufferBarrier.srcAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    // Compute shader buffer read and write
    bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT;
    bufferBarrier.buffer = particlesStorageBuffer.buffer;
    bufferBarrier.size = particlesStorageBuffer.descriptor.range;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
        cmdbuffer,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_FLAGS_NONE,
        0, nullptr,
        1, &bufferBarrier,
        0, nullptr);*/

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, propageFire);
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);
    vkCmdDispatch(cmdbuffer, bGroups, 1, 1);

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, moveBurnPipeline);

    // Dispatch the compute job
    vkCmdDispatch(cmdbuffer, bGroups, 1, 1);

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, updateParticles);

    // Dispatch the compute job
    vkCmdDispatch(cmdbuffer, pGroups, 1, 1);

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, collisionPipeline);

    // Dispatch the compute job
    vkCmdDispatch(cmdbuffer, pGroups, 1, 1);

    /*// Add memory barrier to ensure that compute shader has finished writing to the buffer
    // Without this the (rendering) vertex shader may display incomplete results (partial data from last frame)
    // Compute shader has finished writes to the buffer
    bufferBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    // Vertex shader access (attribute binding)
    bufferBarrier.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    bufferBarrier.buffer = particlesStorageBuffer.buffer;
    bufferBarrier.size = particlesStorageBuffer.descriptor.range;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
        cmdbuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT,
        VK_FLAGS_NONE,
        0, nullptr,
        1, &bufferBarrier,
        0, nullptr);*/
}

void VulkanFire::draw(VkCommandBuffer cmdbuffer)
{
    // Particle system
    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, drawParticles);
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

    VkDeviceSize offsets[1] = { 0 };
    vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &particlesStorageBuffer.buffer, offsets);
    vkCmdDraw(cmdbuffer, computeUbo.particleCount, 1, 0, 0);
}

void VulkanFire::init(VkQueue queue,VkCommandPool cpool, VkRenderPass renderpass,VkDescriptorPool pool, VkDescriptorBufferInfo* bPointsDesc, uint32_t bCount, VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke, std::string path){
    computeUbo.bPointsCount=bCount;
    prepareParticles(queue);
    prepareGrid();
    prepareUniformBuffers();
    prepareComputeLayout(pool,bPointsDesc);
    prepareComputePipelines(path);
    setupDescriptorSet(pool,bPointsDesc,sampler,fire,smoke);
    prepareRenderPipelines(renderpass,path);
    createClickCommand(cpool);
    fillGrid(queue,cpool);
}

float VulkanFire::rnd(float range)
{
    return range * (rand() / double(RAND_MAX));
}

void VulkanFire::initParticle(Particle *particle, int index)
{

    //Body creation
    /*btCollisionShape* groundShape = new btSphereShape(btScalar(particle->size));
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();

    VkParticleMotionState* myMotionState = new VkParticleMotionState(groundTransform,this,index);

    groundShape->calculateLocalInertia(PARTICLE_MASS,localInertia);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(PARTICLE_MASS,myMotionState,groundShape,localInertia);
    particle->body = new btRigidBody(rbInfo,false);

    reset(particle);*/

}


void VulkanFire::reset(Particle *particle){
    /*;

    particle->body->getWorldTransform().setOrigin(btVector3(particle->pos.x, particle->pos.y, particle->pos.z));

    particle->body->setLinearVelocity(btVector3(0.0f, minVel.y + rnd(maxVel.y - minVel.y), 0.0f));*/
}

void VulkanFire::transitionParticle(Particle *particle)
{
    /*switch (particle->type)
    {
    case PARTICLE_TYPE_FLAME:
        // Flame particles have a chance of turning into smoke
        if (rnd(1.0f) < 0.05f)
        {
            particle->alpha = 0.0f;
            particle->color = glm::vec4(0.25f + rnd(0.25f));

            particle->body->getWorldTransform().setOrigin(btVector3(particle->pos.x*0.5f, particle->pos.y, particle->pos.z*0.5f));
            particle->body->setLinearVelocity(btVector3(rnd(1.0f) - rnd(1.0f), (minVel.y * 2) + rnd(maxVel.y - minVel.y), rnd(1.0f) - rnd(1.0f)));
            particle->size = 1.0f + rnd(0.5f);
            particle->rotationSpeed = rnd(1.0f) - rnd(1.0f);
            particle->type = PARTICLE_TYPE_SMOKE;
        }
        else
        {
            reset(particle);
        }
        break;
    case PARTICLE_TYPE_SMOKE:
        // Respawn at end of life
        reset(particle);
        break;
    }*/
}

void VulkanFire::prepareParticles(VkQueue queue)
{

    glm::vec3 minVel = glm::vec3(-3.0f, 0.5f, -3.0f);
    glm::vec3 maxVel = glm::vec3(3.0f, 7.0f, 3.0f);
    std::vector<Particle> particleBuffer;

    particleBuffer.resize(computeUbo.bPointsCount);
    computeUbo.particleCount =computeUbo.bPointsCount;

    for (auto& particle : particleBuffer)
    {
        particle.pos[0]=0;
        particle.pos[1]=0;
        particle.pos[2]=0;
        particle.alpha = 0.75f;//rnd(0.75f);
        particle.size = 0.05f + rnd(0.025f);
        particle.color = glm::vec4(1.0f);
        particle.type = PARTICLE_TYPE_FLAME;
        particle.rotation = rnd(2.0f * M_PI);
        particle.vel.w = rnd(2.0f) - rnd(2.0f);
        particle.vel=glm::vec4(0.0f, minVel.y + rnd(maxVel.y - minVel.y), 0.0f, 0.0f);
    }


    // Setup and fill the compute shader storage buffers for
    // vertex positions and velocities

    uint32_t storageBufferSize = particleBuffer.size() * sizeof(Particle);

    // Staging
    // SSBO is static, copy to device local memory
    // This results in better performance

    struct {
        VkDeviceMemory memory;
        VkBuffer buffer;
    } stagingBuffer;
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
        storageBufferSize,
        particleBuffer.data(),
        &stagingBuffer.buffer,
        &stagingBuffer.memory);
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        storageBufferSize,
        nullptr,
        &particlesStorageBuffer.buffer,
        &particlesStorageBuffer.memory);
    // Copy to staging buffer
    VkCommandBuffer copyCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkBufferCopy copyRegion = {};
    copyRegion.size = storageBufferSize;
    vkCmdCopyBuffer(
        copyCmd,
        stagingBuffer.buffer,
        particlesStorageBuffer.buffer,
        1,
        &copyRegion);

    exampleBase->flushCommandBuffer(copyCmd, queue, true);

    vkFreeMemory(device, stagingBuffer.memory, nullptr);
    vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);

    particlesStorageBuffer.descriptor.range = storageBufferSize;
    particlesStorageBuffer.descriptor.buffer = particlesStorageBuffer.buffer;
    particlesStorageBuffer.descriptor.offset = 0;

    // Binding description
    attributesParticles.bindingDescriptions.resize(1);
    attributesParticles.bindingDescriptions[0] =
        vkTools::initializers::vertexInputBindingDescription(
            VERTEX_BUFFER_BIND_ID,
            sizeof(Particle),
            VK_VERTEX_INPUT_RATE_VERTEX);

    // Attribute descriptions
    // Describes memory layout and shader positions
    // Location 0 : Position
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            0,
            VK_FORMAT_R32G32B32A32_SFLOAT,
            0));
    // Location 1 : Color
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            1,
            VK_FORMAT_R32G32B32A32_SFLOAT,
            sizeof(float) * 4));
    // Location 2 : Alpha
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            2,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 12));
    // Location 3 : Size
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            3,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 13));
    // Location 4 : Rotation
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            4,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 14));
    // Location 5 : Type
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            5,
            VK_FORMAT_R32_SINT,
            sizeof(float) * 15));

    // Assign to vertex buffer
    attributesParticles.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
    attributesParticles.inputState.vertexBindingDescriptionCount = attributesParticles.bindingDescriptions.size();
    attributesParticles.inputState.pVertexBindingDescriptions = attributesParticles.bindingDescriptions.data();
    attributesParticles.inputState.vertexAttributeDescriptionCount = attributesParticles.attributeDescriptions.size();
    attributesParticles.inputState.pVertexAttributeDescriptions = attributesParticles.attributeDescriptions.data();
}


void VulkanFire::prepareGrid(){


    exampleBase->createBuffer(
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        200*200*200*4,
        nullptr,
        &gridStorageBuffer.buffer,
        &gridStorageBuffer.memory);

    gridStorageBuffer.descriptor.range = 200*200*200*4;
    gridStorageBuffer.descriptor.buffer = gridStorageBuffer.buffer;
    gridStorageBuffer.descriptor.offset = 0;
}

void VulkanFire::prepareComputeLayout(VkDescriptorPool descriptorPool, VkDescriptorBufferInfo* burnDescriptor)
{
    // Create compute pipeline
    // Compute pipelines are created separate from graphics pipelines
    // even if they use the same queue

    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        // Binding 0 : Particle storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0),
        // Binding 1 : Burning points storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            1),
        // Binding 1 : Burning points storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            2),
        // Binding 3 : Collision grid storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            3),
        // Binding 4 : Uniform buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            4)
    };

    VkDescriptorSetLayoutCreateInfo descriptorLayout =
        vkTools::initializers::descriptorSetLayoutCreateInfo(
            setLayoutBindings.data(),
            setLayoutBindings.size());

    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device,	&descriptorLayout, nullptr,	&computeDescriptorSetLayout));


    VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
        vkTools::initializers::pipelineLayoutCreateInfo(
            &computeDescriptorSetLayout,
            1);

    VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr,	&computePipelineLayout));

    VkDescriptorSetAllocateInfo allocInfo =
        vkTools::initializers::descriptorSetAllocateInfo(
            descriptorPool,
            &computeDescriptorSetLayout,
            1);

    VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &computeDescriptorSet));

    std::vector<VkWriteDescriptorSet> computeWriteDescriptorSets =
    {
        // Binding 0 : Particle position storage buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            0,
            &particlesStorageBuffer.descriptor),
        // Binding 1 : Burning points storage buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            1,
            &burnDescriptor[0]),
        // Binding 2 : Objects Models
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            2,
            &burnDescriptor[1]),
        // Binding 3 : Collision grid storage buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            3,
            &gridStorageBuffer.descriptor),
        // Binding 4 : Uniform buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            4,
            &computeUniformBuffer.descriptor)
    };
    vkUpdateDescriptorSets(device, computeWriteDescriptorSets.size(), computeWriteDescriptorSets.data(), 0, NULL);
}

void VulkanFire::prepareComputePipelines(std::string assetPath){

    VkPipelineCache pipelineCache;

    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VK_CHECK_RESULT(vkCreatePipelineCache(device, &pipelineCacheCreateInfo, nullptr, &pipelineCache));


    VkComputePipelineCreateInfo computePipelineCreateInfo =
        vkTools::initializers::computePipelineCreateInfo(
            computePipelineLayout,
            0);

    // Create upadate particle pipeline
    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/updateParticle.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &updateParticles));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/propageFire.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &propageFire));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/clickFire.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &clickFire));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/moveBurn.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &moveBurnPipeline));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/particleCollision.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &collisionPipeline));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/fillGrid.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &fillGridPipeline));
}

//Création de la commande permet de partir un feu avec clic molette
void VulkanFire::createClickCommand(VkCommandPool cmdPool){

    int bGroups  = 1;
    while(bGroups*512<computeUbo.bPointsCount){
        bGroups++;
    }
    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vkTools::initializers::commandBufferAllocateInfo(
            cmdPool,
            VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            1);

    VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &clickCmd));

    VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

    vkBeginCommandBuffer(clickCmd,&cmdBufInfo);

    vkCmdBindPipeline(clickCmd, VK_PIPELINE_BIND_POINT_COMPUTE, clickFire);
    vkCmdBindDescriptorSets(clickCmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);

    // Dispatch the compute job
    vkCmdDispatch(clickCmd, bGroups, 1, 1);

    vkEndCommandBuffer(clickCmd);
}

//Exécute la commande pour partir un feu avec la molette
void VulkanFire::cliked(VkQueue queue, glm::vec4 pos){
    computeUbo.clickPos = pos;

    memcpy(computeUniformBuffer.mapped, &computeUbo, sizeof(computeUbo));
    VkSubmitInfo submitInfo = vkTools::initializers::submitInfo();
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &clickCmd;

    // Submit to queue
    vkDeviceWaitIdle(device);
    vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
}

void VulkanFire::prepareRenderLayout(VkDescriptorPool descriptorPool){
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
    {
        // Binding 0 : Vertex shader uniform buffer
        vkTools::initializers::descriptorSetLayoutBinding(
        VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            VK_SHADER_STAGE_VERTEX_BIT,
            0),
        // Binding 1 : Fragment shader image sampler
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            1),
        // Binding 2 : Fragment shader image sampler
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            2)
    };

    VkDescriptorSetLayoutCreateInfo descriptorLayout =
        vkTools::initializers::descriptorSetLayoutCreateInfo(
            setLayoutBindings.data(),
            setLayoutBindings.size());

    VkResult err = vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout);
    assert(!err);

    VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
        vkTools::initializers::pipelineLayoutCreateInfo(
            &descriptorSetLayout,
            1);

    err = vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
    assert(!err);


}

void VulkanFire::prepareRenderPipelines(VkRenderPass renderPass,std::string assetPath){

    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
        vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
            VK_PRIMITIVE_TOPOLOGY_POINT_LIST,
            0,
            VK_FALSE);

    VkPipelineRasterizationStateCreateInfo rasterizationState =
        vkTools::initializers::pipelineRasterizationStateCreateInfo(
            VK_POLYGON_MODE_FILL,
            VK_CULL_MODE_BACK_BIT,
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

    // Load shaders
    std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

    shaderStages[0] = exampleBase->loadShader(assetPath + "shaders/particlefire/particle.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
    shaderStages[1] = exampleBase->loadShader(assetPath + "shaders/particlefire/particle.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

    VkGraphicsPipelineCreateInfo pipelineCreateInfo =
        vkTools::initializers::pipelineCreateInfo(
            pipelineLayout,
            renderPass,
            0);

    pipelineCreateInfo.pVertexInputState = &attributesParticles.inputState;
    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.stageCount = shaderStages.size();
    pipelineCreateInfo.pStages = shaderStages.data();

    depthStencilState.depthWriteEnable = VK_FALSE;

    // Premulitplied alpha
    blendAttachmentState.blendEnable = VK_TRUE;
    blendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
    blendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    blendAttachmentState.alphaBlendOp = VK_BLEND_OP_ADD;
    blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineCache pipelineCache;

    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VK_CHECK_RESULT(vkCreatePipelineCache(device, &pipelineCacheCreateInfo, nullptr, &pipelineCache));

    VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &drawParticles);
    assert(!err);
}


/*void VulkanFire::updateParticle(btVector3 ori, int index)
{
    Particle* particle = &particleBuffer.at(index);
    float particleTimer = 0.0003 * 0.45f;
    switch (particle->type)
    {
    case PARTICLE_TYPE_FLAME:
        particle->pos =glm::vec4(ori.x(),ori.y(),ori.z(),0.0f);
        particle->alpha += particleTimer * 2.5f;
        particle->size -= particleTimer * 0.5f;
        break;
    case PARTICLE_TYPE_SMOKE:
        particle->alpha += particleTimer * 1.25f;
        particle->size += particleTimer * 0.125f;
        particle->color -= particleTimer * 0.05f;
        break;
    }
    particle->rotation += particleTimer * particle->rotationSpeed;
    // Transition particle state
    if (particle->alpha > 2.0f)
    {
        transitionParticle(particle);
    }
    size_t size = particleBuffer.size() * sizeof(Particle);
    memcpy(particleVkBuffer.mappedMemory, particleBuffer.data(), size);
}*/

void VulkanFire::setupDescriptorSet(VkDescriptorPool pool,VkDescriptorBufferInfo* infos, VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke)
{
    prepareRenderLayout(pool);
    VkDescriptorSetAllocateInfo allocInfo =
        vkTools::initializers::descriptorSetAllocateInfo(
            pool,
            &descriptorSetLayout,
            1);
    VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
    assert(!vkRes);
    // Image descriptor for the color map texture
    VkDescriptorImageInfo texDescriptorSmoke =
        vkTools::initializers::descriptorImageInfo(
            sampler,
            smoke.view,
            VK_IMAGE_LAYOUT_GENERAL);
    VkDescriptorImageInfo texDescriptorFire =
        vkTools::initializers::descriptorImageInfo(
            sampler,
            fire.view,
            VK_IMAGE_LAYOUT_GENERAL);

    std::vector<VkWriteDescriptorSet> writeDescriptorSets =
    {
        // Binding 0 : Vertex shader uniform buffer
        vkTools::initializers::writeDescriptorSet(
        descriptorSet,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            0,
            &infos[2]),
        // Binding 1 : Smoke texture
        vkTools::initializers::writeDescriptorSet(
            descriptorSet,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1,
            &texDescriptorSmoke),
        // Binding 1 : Fire texture array
        vkTools::initializers::writeDescriptorSet(
            descriptorSet,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            2,
            &texDescriptorFire)
    };

    vkUpdateDescriptorSets(device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);
}

void VulkanFire::prepareUniformBuffers()
{
    //computeUbo.deltaT=0.0f;

    // Compute shader uniform buffer block
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        sizeof(computeUbo),
        &computeUbo,
        &computeUniformBuffer.buffer,
        &computeUniformBuffer.memory,
        &computeUniformBuffer.descriptor);

    // Map for host access
    vkTools::checkResult(vkMapMemory(device, computeUniformBuffer.memory, 0, sizeof(computeUbo), 0, (void **)&computeUniformBuffer.mapped));
}


void VulkanFire::updateTime(float frameTimer)
{
    computeUbo.deltaT= frameTimer;//glm::vec4(1,6,1,frameTimer);
    memcpy(computeUniformBuffer.mapped, &computeUbo, sizeof(computeUbo));
}

void VulkanFire::addToWorld(btDiscreteDynamicsWorld* dw){
    /*for (auto& particle : particleBuffer)
    {
        dw->addRigidBody(particle.body);
        particle.body->setGravity(btVector3(0.0f,0.0f,0.0f));
    }*/
}

void VulkanFire::fillGrid(VkQueue queue, VkCommandPool cmdPool){

    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vkTools::initializers::commandBufferAllocateInfo(
            cmdPool,
            VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            1);

    VkCommandBuffer fillCmd;
    VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &fillCmd));

    VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

    vkBeginCommandBuffer(fillCmd,&cmdBufInfo);

    vkCmdBindPipeline(fillCmd, VK_PIPELINE_BIND_POINT_COMPUTE, fillGridPipeline);
    vkCmdBindDescriptorSets(fillCmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);

    // Dispatch the compute job
    vkCmdDispatch(fillCmd, 15625, 1, 1);

    vkEndCommandBuffer(fillCmd);

    VkSubmitInfo submitInfo = vkTools::initializers::submitInfo();
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &fillCmd;

    vkQueueSubmit(queue,1,&submitInfo,VK_NULL_HANDLE);
}
