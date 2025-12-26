# Quickstart Guide: UI Improvement for Book Frontend

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git
- A modern web browser for testing

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd book-frontend
```

### 2. Install Dependencies
```bash
npm install
# OR if using yarn
yarn install
```

### 3. Start Development Server
```bash
npm start
# OR if using yarn
yarn start
```

This will start the Docusaurus development server at `http://localhost:3000`

## Key Configuration Files

### Docusaurus Configuration
- File: `docusaurus.config.js`
- Purpose: Main configuration for the Docusaurus site including theme settings, plugins, and metadata

### Sidebar Configuration
- File: `sidebars.js`
- Purpose: Defines the navigation structure and hierarchy for the documentation

### Custom Styles
- File: `src/css/custom.css`
- Purpose: Custom CSS overrides for the UI improvements
- File: `src/css/theme.css`
- Purpose: Theme-specific styling for colors, typography, and layout

## Making UI Changes

### Custom Components
1. Create new components in `src/components/`
2. Use React functional components with TypeScript/JavaScript
3. Follow Docusaurus component patterns for consistency

### Styling
1. Add custom styles to `src/css/custom.css`
2. Use CSS custom properties (variables) for consistent theming
3. Implement responsive design using media queries
4. Ensure accessibility with proper contrast ratios

### Navigation Updates
1. Modify navigation in `docusaurus.config.js`
2. Update sidebar structure in `sidebars.js`
3. Test navigation flows to ensure 2-click maximum requirement

## Testing

### Local Testing
1. Run development server: `npm start`
2. Navigate to different pages to verify UI consistency
3. Test responsive behavior by resizing browser window
4. Verify accessibility using browser developer tools

### Build Testing
```bash
npm run build
npm run serve
```
This will build the static site and serve it locally to test production build.

## Deployment

### GitHub Pages Deployment
```bash
GIT_USER=<Your GitHub username> npm run deploy
```

### Environment Variables
- `GIT_USER`: GitHub username for deployment
- `USE_SSH`: Set to `true` to use SSH for Git operations

## Troubleshooting

### Common Issues
- **Styles not applying**: Clear browser cache and restart development server
- **Responsive design not working**: Check media query syntax and breakpoints
- **Navigation not updating**: Verify changes in both `docusaurus.config.js` and `sidebars.js`

### Development Tips
- Use hot reloading during development (changes reflect immediately)
- Test on multiple browsers to ensure compatibility
- Use browser dev tools to inspect elements and debug CSS
- Check Lighthouse scores to maintain performance standards