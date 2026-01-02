# Deployment Configuration for GitHub Pages

## GitHub Actions Workflow

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

## Docusaurus Configuration Update

Update `docusaurus.config.js` for GitHub Pages:

```js
const config = {
  // ... other configuration
  url: 'https://your-username.github.io',
  baseUrl: '/ai-native-textbook-platform/',
  projectName: 'ai-native-textbook-platform', // Usually your repo name
  organizationName: 'your-username', // Usually your GitHub org/user name
  trailingSlash: false,
  // ... rest of configuration
};
```

## Deployment Script

Create `deploy.sh`:

```bash
#!/bin/bash
set -e

npm run build

# Navigate to the build output directory
cd build

# Create a temporary git repository
git init
git add .
git commit -m "Deploy to GitHub Pages"

# Push to the gh-pages branch
git push -f https://github.com/your-username/ai-native-textbook-platform.git master:gh-pages

cd -
```

## Environment Configuration

Create `.env` for deployment variables:

```
GITHUB_TOKEN=your_github_token_here
```

## README.md Deployment Section

Add to README.md:

```markdown
## Deployment

This website is built using Docusaurus 3 and deployed to GitHub Pages.

### Local Development

```
npm install
npm start
```

### Deployment to GitHub Pages

The site is automatically deployed via GitHub Actions when changes are pushed to the main branch.

To deploy manually:
```
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```

The website will be available at: https://your-username.github.io/ai-native-textbook-platform/
```