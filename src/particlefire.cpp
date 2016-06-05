#include "particlefire.h"

VulkanFire::VulkanFire(VkDevice device, VulkanExampleBase *example,glm::vec3 pos)
{
    this->device = device;
    this->exampleBase=example;
    emitterPos = pos;
    prepareParticles();
    prepareUniformBuffers();

}

VulkanFire::~VulkanFire()
{
    // Clean up used Vulkan resources
    // Note : Inherited destructor cleans up resources stored in base class

    vkUnmapMemory(device, particleVkBuffer.memory);
    vkDestroyBuffer(device, particleVkBuffer.buffer, nullptr);
    vkFreeMemory(device, particleVkBuffer.memory, nullptr);

    vkDestroyBuffer(device, uniformData.buffer, nullptr);
    vkFreeMemory(device, uniformData.memory, nullptr);
}

void VulkanFire::draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)
{
    // Particle system
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

    VkDeviceSize offsets[1] = { 0 };
    vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &particleVkBuffer.buffer, offsets);
    vkCmdDraw(cmdbuffer, PARTICLE_COUNT, 1, 0, 0);
}

float VulkanFire::rnd(float range)
{
    return range * (rand() / double(RAND_MAX));
}

void VulkanFire::initParticle(Particle *particle, glm::vec3 emitterPos)
{
    particle->vel = glm::vec4(0.0f, minVel.y + rnd(maxVel.y - minVel.y), 0.0f, 0.0f);
    particle->alpha = rnd(0.75f);
    particle->size = 0.05f + rnd(0.025f);
    particle->color = glm::vec4(1.0f);
    particle->type = PARTICLE_TYPE_FLAME;
    particle->rotation = rnd(2.0f * M_PI);
    particle->rotationSpeed = rnd(2.0f) - rnd(2.0f);

    // Get random sphere point
    float theta = rnd(2 * M_PI);
    float phi = rnd(M_PI) - M_PI / 2;
    float r = rnd(FLAME_RADIUS);

    particle->pos.x = r * cos(theta) * cos(phi);
    particle->pos.y = r * sin(phi);
    particle->pos.z = r * sin(theta) * cos(phi);

    particle->pos += glm::vec4(emitterPos, 0.0f);
}

void VulkanFire::transitionParticle(Particle *particle)
{
    switch (particle->type)
    {
    case PARTICLE_TYPE_FLAME:
        // Flame particles have a chance of turning into smoke
        if (rnd(1.0f) < 0.05f)
        {
            particle->alpha = 0.0f;
            particle->color = glm::vec4(0.25f + rnd(0.25f));
            particle->pos.x *= 0.5f;
            particle->pos.z *= 0.5f;
            particle->vel = glm::vec4(rnd(1.0f) - rnd(1.0f), (minVel.y * 2) + rnd(maxVel.y - minVel.y), rnd(1.0f) - rnd(1.0f), 0.0f);
            particle->size = 1.0f + rnd(0.5f);
            particle->rotationSpeed = rnd(1.0f) - rnd(1.0f);
            particle->type = PARTICLE_TYPE_SMOKE;
        }
        else
        {
            initParticle(particle, emitterPos);
        }
        break;
    case PARTICLE_TYPE_SMOKE:
        // Respawn at end of life
        initParticle(particle, emitterPos);
        break;
    }
}

void VulkanFire::prepareParticles()
{
    particleBuffer.resize(PARTICLE_COUNT);
    for (auto& particle : particleBuffer)
    {
        initParticle(&particle, emitterPos);
        particle.alpha = 1.0f - (abs(particle.pos.y) / (FLAME_RADIUS * 2.0f));
    }

    particleVkBuffer.size = particleBuffer.size() * sizeof(Particle);

    exampleBase->createBuffer(
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        particleVkBuffer.size,
        particleBuffer.data(),
        &particleVkBuffer.buffer,
        &particleVkBuffer.memory);

    // Map the memory and store the pointer for reuse
    VkResult err = vkMapMemory(device, particleVkBuffer.memory, 0, particleVkBuffer.size, 0, &particleVkBuffer.mappedMemory);
    assert(!err);
}

void VulkanFire::updateParticles(float frameTimer)
{
    float particleTimer = frameTimer * 0.45f;
    for (auto& particle : particleBuffer)
    {
        switch (particle.type)
        {
        case PARTICLE_TYPE_FLAME:
            particle.pos.y += particle.vel.y * particleTimer * 3.5f;
            particle.alpha += particleTimer * 2.5f;
            particle.size -= particleTimer * 0.5f;
            break;
        case PARTICLE_TYPE_SMOKE:
            particle.pos += particle.vel * frameTimer * 1.0f;
            particle.alpha += particleTimer * 1.25f;
            particle.size += particleTimer * 0.125f;
            particle.color -= particleTimer * 0.05f;
            break;
        }
        particle.rotation += particleTimer * particle.rotationSpeed;
        // Transition particle state
        if (particle.alpha > 2.0f)
        {
            transitionParticle(&particle);
        }
    }
    size_t size = particleBuffer.size() * sizeof(Particle);
    memcpy(particleVkBuffer.mappedMemory, particleBuffer.data(), size);
}

void VulkanFire::setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout,VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke)
{
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
            &uniformData.descriptor),
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
    ubo.model = glm::mat4();
    // Vertex shader uniform buffer block
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        sizeof(ubo),
        &ubo,
        &uniformData.buffer,
        &uniformData.memory,
        &uniformData.descriptor);
}
