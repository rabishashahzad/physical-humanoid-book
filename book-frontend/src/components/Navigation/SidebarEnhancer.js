import React, { useState } from 'react';
import clsx from 'clsx';

// Enhanced sidebar component to improve navigation experience
export default function SidebarEnhancer({children}) {
  const [isExpanded, setIsExpanded] = useState(true);

  const toggleSidebar = () => {
    setIsExpanded(!isExpanded);
  };

  return (
    <div className={clsx('sidebar-enhancer', {'sidebar-enhancer--collapsed': !isExpanded})}>
      <button
        className="sidebar-toggle-button"
        onClick={toggleSidebar}
        aria-label={isExpanded ? "Collapse sidebar" : "Expand sidebar"}
        aria-expanded={isExpanded}
      >
        <svg className="sidebar-toggle-icon" width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d={isExpanded ? "M15 18L9 12L15 6" : "M9 18L15 12L9 6"} stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </button>
      <div className="sidebar-content">
        {children}
      </div>
      <style jsx>{`
        .sidebar-enhancer {
          position: relative;
          display: flex;
          flex-direction: column;
        }

        .sidebar-toggle-button {
          position: absolute;
          top: var(--ifm-spacing-4);
          right: var(--ifm-spacing-2);
          z-index: 101;
          background: var(--ifm-color-background);
          border: 1px solid var(--ifm-color-border);
          border-radius: var(--ifm-spacing-2);
          padding: var(--ifm-spacing-2);
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
          transition: var(--ifm-transition-all);
          color: var(--ifm-color-text);
        }

        .sidebar-toggle-button:hover {
          background-color: var(--ifm-color-background-alt);
          box-shadow: var(--ifm-shadow-sm);
        }

        .sidebar-toggle-icon {
          width: 1.25rem;
          height: 1.25rem;
          transition: var(--ifm-transition-all);
        }

        .sidebar-content {
          transition: var(--ifm-transition-all);
        }

        /* Add visual enhancements to sidebar */
        .sidebar-enhancer .menu__list-item {
          transition: var(--ifm-transition-all);
          border-radius: var(--ifm-spacing-1);
          margin: var(--ifm-spacing-05) 0;
        }

        .sidebar-enhancer .menu__list-item:hover {
          background-color: var(--ifm-color-background-alt);
          margin: var(--ifm-spacing-05) var(--ifm-spacing-1);
          padding: var(--ifm-spacing-2) var(--ifm-spacing-3);
        }

        .sidebar-enhancer .menu__link {
          transition: var(--ifm-transition-all);
          border-radius: var(--ifm-spacing-1);
          padding: var(--ifm-spacing-2) var(--ifm-spacing-3);
        }

        .sidebar-enhancer .menu__link:hover {
          background-color: var(--ifm-color-primary-lightest);
          color: var(--ifm-color-primary-dark);
        }

        .sidebar-enhancer .menu__link--active {
          background-color: var(--ifm-color-primary-lighter);
          color: var(--ifm-color-primary-darker);
          font-weight: var(--ifm-font-weight-semibold);
        }

        .sidebar-enhancer .menu__list-item-collapsible {
          border-radius: var(--ifm-spacing-1);
        }

        .sidebar-enhancer .menu__list-item-collapsible:hover {
          background-color: var(--ifm-color-background-alt);
        }
      `}</style>
    </div>
  );
}