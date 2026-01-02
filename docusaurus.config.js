// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Book',
  tagline: 'An AI-Native textbook on bridging artificial intelligence and robotics',
  favicon: 'img/favicon.ico',

  // Required fields (from docs)
  url: 'https://azfar-sohail.github.io',

  // ✅ Environment-aware baseUrl
  baseUrl: process.env.VERCEL === '1' ? '/' : '/book_writting/',

  organizationName: 'Azfar-sohail',
  projectName: 'book_writting',

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
          routeBasePath: '/', // docs = homepage
          editUrl:
            'https://github.com/Azfar-sohail/book_writting/tree/main/',
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
          href: 'https://github.com/Azfar-sohail/book_writting',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: undefined,
  },
};

export default config;
