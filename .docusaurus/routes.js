import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug', '0f7'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/config',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/config', '1ce'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/content',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/content', '72c'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/globalData', '749'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/metadata', '9ce'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/registry', 'c4c'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-native-textbook-platform/__docusaurus/debug/routes', '89a'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/search',
    component: ComponentCreator('/ai-native-textbook-platform/search', '70c'),
    exact: true
  },
  {
    path: '/ai-native-textbook-platform/',
    component: ComponentCreator('/ai-native-textbook-platform/', 'ad1'),
    routes: [
      {
        path: '/ai-native-textbook-platform/',
        component: ComponentCreator('/ai-native-textbook-platform/', '3d1'),
        routes: [
          {
            path: '/ai-native-textbook-platform/',
            component: ComponentCreator('/ai-native-textbook-platform/', 'ece'),
            routes: [
              {
                path: '/ai-native-textbook-platform/part1/',
                component: ComponentCreator('/ai-native-textbook-platform/part1/', '1bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/ai_meets_the_physical_world',
                component: ComponentCreator('/ai-native-textbook-platform/part1/ai_meets_the_physical_world', '7e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/basics_of_robotics_systems',
                component: ComponentCreator('/ai-native-textbook-platform/part1/basics_of_robotics_systems', 'cd6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/introduction_to_ros2',
                component: ComponentCreator('/ai-native-textbook-platform/part1/introduction_to_ros2', '4c0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/nodes_topics_and_messages',
                component: ComponentCreator('/ai-native-textbook-platform/part1/nodes_topics_and_messages', '945'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/services_actions_and_parameters',
                component: ComponentCreator('/ai-native-textbook-platform/part1/services_actions_and_parameters', '2dd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/simulation_before_reality',
                component: ComponentCreator('/ai-native-textbook-platform/part1/simulation_before_reality', 'e5b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/software_stack_for_humanoid_robots',
                component: ComponentCreator('/ai-native-textbook-platform/part1/software_stack_for_humanoid_robots', '3c0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/urdf_robot_description',
                component: ComponentCreator('/ai-native-textbook-platform/part1/urdf_robot_description', '0e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/what_is_physical_ai',
                component: ComponentCreator('/ai-native-textbook-platform/part1/what_is_physical_ai', 'b68'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part1/writing_ros2_nodes_with_rclpy',
                component: ComponentCreator('/ai-native-textbook-platform/part1/writing_ros2_nodes_with_rclpy', '8ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/',
                component: ComponentCreator('/ai-native-textbook-platform/part2/', '346'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/autonomous_decision_making',
                component: ComponentCreator('/ai-native-textbook-platform/part2/autonomous_decision_making', '6bf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/capstone_the_autonomous_humanoid',
                component: ComponentCreator('/ai-native-textbook-platform/part2/capstone_the_autonomous_humanoid', '067'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/gazebo_physics_simulation',
                component: ComponentCreator('/ai-native-textbook-platform/part2/gazebo_physics_simulation', '67f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/isaac_ros_navigation_nav2',
                component: ComponentCreator('/ai-native-textbook-platform/part2/isaac_ros_navigation_nav2', '4ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/language_to_action_planning',
                component: ComponentCreator('/ai-native-textbook-platform/part2/language_to_action_planning', '8e6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/nvidia_isaac_sim',
                component: ComponentCreator('/ai-native-textbook-platform/part2/nvidia_isaac_sim', 'c0d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/sensors_in_simulation',
                component: ComponentCreator('/ai-native-textbook-platform/part2/sensors_in_simulation', 'cc6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/unity_for_human_robot_interaction',
                component: ComponentCreator('/ai-native-textbook-platform/part2/unity_for_human_robot_interaction', '965'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/voice_to_text_with_whisper',
                component: ComponentCreator('/ai-native-textbook-platform/part2/voice_to_text_with_whisper', 'fe3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/part2/what_is_vision_language_action',
                component: ComponentCreator('/ai-native-textbook-platform/part2/what_is_vision_language_action', 'c2b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-native-textbook-platform/rag_personalization_translation',
                component: ComponentCreator('/ai-native-textbook-platform/rag_personalization_translation', '487'),
                exact: true
              },
              {
                path: '/ai-native-textbook-platform/',
                component: ComponentCreator('/ai-native-textbook-platform/', '6e4'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
