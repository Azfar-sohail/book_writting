// @ts-check
// Docusaurus config for a single Book landing page

import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Book',
  tagline: 'An AI-Native textbook on bridging artificial intelligence and robotics',
  favicon: 'img/favicon.ico',

  // ✅ GitHub Pages URL (user site)
  url: 'https://azfar-sohail.github.io',

  // ✅ MUST match repo name exactly
  baseUrl: '/book-writting/',

  // ✅ GitHub org/user
  organizationName: 'Azfar-sohail',

  // ✅ Repository name
  projectName: 'book-writting',

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
          routeBasePath: '/', // Docs = landing page
          editUrl:
            'https://github.com/Azfar-sohail/book-writting/tree/main/',
        },
        blog: false,
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

    navbar: {
      title: 'Book',
      items: [
        {
          href: 'https://github.com/Azfar-sohail/book-writting',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: undefined,

    // ❌ Disable Algolia unless you configured it
    algolia: undefined,
  },
};

export default config;
