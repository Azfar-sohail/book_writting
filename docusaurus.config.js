// @ts-check
// Docusaurus config for a single Book landing page

import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Book', // Landing page title
  tagline: 'An AI-Native textbook on bridging artificial intelligence and robotics',
  favicon: 'img/favicon.ico',

  url: 'https://your-username.github.io',
  baseUrl: '/ai-native-textbook-platform/',

  organizationName: 'your-organization',
  projectName: 'physical-ai-humanoid-robots',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-username/physical-ai-humanoid-robots/tree/main/',
          routeBasePath: '/', // Make docs the landing page
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },

    // Navbar configuration: single title "Book"
    navbar: {
      title: 'Book', // Updated title
      logo: undefined, // Remove logo completely
      items: [
        {
          href: 'https://github.com/your-username/physical-ai-humanoid-robots',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: undefined, // No footer

    algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'physical-ai-humanoid-robots',
      contextualSearch: true,
      replaceSearchResultPathname: { from: '/docs/', to: '/' },
      searchPagePath: 'search',
    },
  },
};

export default config;
