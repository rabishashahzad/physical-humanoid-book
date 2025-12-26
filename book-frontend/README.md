# Book Frontend - Enhanced UI

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator with enhanced UI/UX features.

## Features

- **Modern UI Design**: Clean, professional interface with improved typography and color scheme
- **Enhanced Navigation**: Intuitive navigation with search functionality and optimized sidebar structure
- **Fully Responsive**: Adapts seamlessly to desktop, tablet, and mobile devices
- **Accessibility Focused**: WCAG AA compliant with keyboard navigation and screen reader support
- **Performance Optimized**: Fast loading times with optimized assets and efficient code

## Installation

```bash
npm install
```

## Local Development

```bash
npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## UI Improvements

The site includes the following UI enhancements:

- **Color Scheme**: Modern color palette with proper contrast ratios
- **Typography**: Enhanced font system with improved readability
- **Layout**: CSS Grid and Flexbox based layouts for better structure
- **Navigation**: Improved navigation structure with 2-click access to content
- **Responsive Design**: Carefully crafted breakpoints for all device sizes
- **Accessibility**: Focus indicators, screen reader support, and touch targets
- **Performance**: Optimized images and assets for faster loading
