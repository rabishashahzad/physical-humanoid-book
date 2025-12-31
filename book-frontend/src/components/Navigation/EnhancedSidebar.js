/**
 * Enhanced Sidebar Component for Docusaurus
 * Improves sidebar navigation with better organization and user experience
 */
import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useThemeConfig } from '@docusaurus/theme-common';
import clsx from 'clsx';

const EnhancedSidebar = () => {
  const location = useLocation();
  const themeConfig = useThemeConfig();
  const [openCategories, setOpenCategories] = useState({});
  const [searchQuery, setSearchQuery] = useState('');

  // Get sidebar data
  const sidebar = themeConfig.navbar.items.find(item => item.type === 'docSidebar');

  // Function to toggle category expansion
  const toggleCategory = (categoryId) => {
    setOpenCategories(prev => ({
      ...prev,
      [categoryId]: !prev[categoryId]
    }));
  };

  // Enhanced sidebar navigation items
  const enhancedNavItems = [
    {
      title: 'Getting Started',
      items: [
        { label: 'Introduction', to: '/docs/intro', icon: '📘' },
        { label: 'Installation', to: '/docs/installation', icon: '⚙️' },
        { label: 'Configuration', to: '/docs/configuration', icon: '🔧' },
        { label: 'Quick Start', to: '/docs/quick-start', icon: '⚡' }
      ]
    },
    {
      title: 'Core Concepts',
      items: [
        { label: 'Architecture', to: '/docs/architecture', icon: '🏗️' },
        { label: 'Components', to: '/docs/components', icon: '🧩' },
        { label: 'API Reference', to: '/docs/api', icon: '📚' },
        { label: 'Best Practices', to: '/docs/best-practices', icon: '✨' }
      ]
    },
    {
      title: 'Guides',
      items: [
        { label: 'Tutorials', to: '/docs/category/tutorials', icon: '🎯' },
        { label: 'How-to Guides', to: '/docs/category/how-to-guides', icon: '📝' },
        { label: 'Troubleshooting', to: '/docs/troubleshooting', icon: '🛠️' },
        { label: 'FAQ', to: '/docs/faq', icon: '❓' }
      ]
    },
    {
      title: 'Resources',
      items: [
        { label: 'Examples', to: '/docs/examples', icon: '💡' },
        { label: 'Templates', to: '/docs/templates', icon: '📄' },
        { label: 'Community', to: '/community', icon: '👥' },
        { label: 'Releases', to: '/releases', icon: '📦' }
      ]
    }
  ];

  // Filter items based on search query
  const filteredNavItems = enhancedNavItems.map(category => ({
    ...category,
    items: category.items.filter(item =>
      item.label.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.to.toLowerCase().includes(searchQuery.toLowerCase())
    )
  })).filter(category => category.items.length > 0);

  return (
    <div className="enhanced-sidebar">
      {/* Search Bar */}
      <div className="enhanced-sidebar-search">
        <input
          type="text"
          placeholder="Search documentation..."
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          className="enhanced-sidebar-search-input"
          aria-label="Search documentation"
        />
        {searchQuery && (
          <button
            onClick={() => setSearchQuery('')}
            className="enhanced-sidebar-search-clear"
            aria-label="Clear search"
          >
            ✕
          </button>
        )}
      </div>

      {/* Navigation Categories */}
      {filteredNavItems.map((category, index) => (
        <div key={index} className="enhanced-sidebar-category">
          <div
            className="enhanced-sidebar-category-header"
            onClick={() => toggleCategory(category.title)}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                toggleCategory(category.title);
              }
            }}
          >
            <h3 className="enhanced-sidebar-category-title">
              {category.title}
            </h3>
            <span className={`enhanced-sidebar-chevron ${openCategories[category.title] ? 'expanded' : ''}`}>
              ▼
            </span>
          </div>

          {openCategories[category.title] && (
            <ul className="enhanced-sidebar-items">
              {category.items.map((item, itemIndex) => (
                <li key={itemIndex} className="enhanced-sidebar-item">
                  <a
                    href={item.to}
                    className={clsx(
                      'enhanced-sidebar-link',
                      location.pathname === item.to && 'enhanced-sidebar-link--active'
                    )}
                  >
                    <span className="enhanced-sidebar-link-icon">{item.icon}</span>
                    <span className="enhanced-sidebar-link-label">{item.label}</span>
                  </a>
                </li>
              ))}
            </ul>
          )}
        </div>
      ))}

      {/* Quick Links */}
      <div className="enhanced-sidebar-quick-links">
        <h4 className="enhanced-sidebar-quick-links-title">Quick Links</h4>
        <div className="enhanced-sidebar-quick-links-grid">
          <a href="/docs/intro" className="enhanced-sidebar-quick-link">
            <span className="enhanced-sidebar-quick-link-icon">🚀</span>
            <span className="enhanced-sidebar-quick-link-label">Getting Started</span>
          </a>
          <a href="/docs/api" className="enhanced-sidebar-quick-link">
            <span className="enhanced-sidebar-quick-link-icon">⚙️</span>
            <span className="enhanced-sidebar-quick-link-label">API Reference</span>
          </a>
          <a href="/community" className="enhanced-sidebar-quick-link">
            <span className="enhanced-sidebar-quick-link-icon">💬</span>
            <span className="enhanced-sidebar-quick-link-label">Community</span>
          </a>
        </div>
      </div>
    </div>
  );
};

export default EnhancedSidebar;