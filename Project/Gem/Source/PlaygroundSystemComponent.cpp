/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PlaygroundSystemComponent.h"
#include <AzCore/Component/TickBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace Playground
{
    void PlaygroundSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<PlaygroundSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<PlaygroundSystemComponent>("Playground", "Playground system component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<PlaygroundRequestBus>("PlaygroundRequestBus");
        }
    }

    void PlaygroundSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("PlaygroundService"));
    }

    void PlaygroundSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("PlaygroundService"));
    }

    void PlaygroundSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void PlaygroundSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    PlaygroundSystemComponent::PlaygroundSystemComponent()
    {
    }

    PlaygroundSystemComponent::~PlaygroundSystemComponent()
    {
    }

    void PlaygroundSystemComponent::Activate()
    {
        PlaygroundRequestBus::Handler::BusConnect();
    }

    void PlaygroundSystemComponent::Deactivate()
    {
        PlaygroundRequestBus::Handler::BusDisconnect();
    }
} // namespace Playground
