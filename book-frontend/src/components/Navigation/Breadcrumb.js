import React from 'react';
import { useBreadcrumb } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

export default function Breadcrumb() {
  const { breadcrumbs } = useBreadcrumb();

  if (breadcrumbs.length <= 1) {
    // Don't show breadcrumb if there's only one item (home)
    return null;
  }

  return (
    <nav className="breadcrumb-nav" aria-label="Breadcrumb">
      <ol className="breadcrumb">
        {breadcrumbs.map((item, idx) => {
          const isLast = idx === breadcrumbs.length - 1;

          return (
            <li key={idx} className={clsx('breadcrumb__item', {
              'breadcrumb__item--active': isLast,
            })}>
              {isLast ? (
                <span className="breadcrumb__link breadcrumb__link--active" aria-current="page">{item.label}</span>
              ) : (
                <Link to={item.href} className="breadcrumb__link" aria-label={`Go to ${item.label}`}>
                  {item.label}
                </Link>
              )}
              {!isLast && (
                <span className="breadcrumb__separator" aria-hidden="true">
                  <svg className="breadcrumb__separator-icon" width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M6 12L10 8L6 4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </span>
              )}
            </li>
          );
        })}
      </ol>
      <style jsx>{`
        .breadcrumb-nav {
          margin-bottom: var(--ifm-spacing-4);
          padding: var(--ifm-spacing-3) 0;
        }

        .breadcrumb {
          display: flex;
          flex-wrap: wrap;
          padding: 0;
          margin: 0;
          list-style: none;
          align-items: center;
          font-size: var(--ifm-font-size-sm);
        }

        .breadcrumb__item {
          display: flex;
          align-items: center;
          padding: 0;
          color: var(--ifm-color-text-secondary);
        }

        .breadcrumb__item + .breadcrumb__item::before {
          content: '';
        }

        .breadcrumb__link {
          color: var(--ifm-color-primary);
          text-decoration: none;
          transition: var(--ifm-transition-colors);
          padding: var(--ifm-spacing-1) var(--ifm-spacing-2);
          border-radius: var(--ifm-spacing-1);
        }

        .breadcrumb__link:hover {
          color: var(--ifm-color-primary-dark);
          background-color: var(--ifm-color-background-alt);
          text-decoration: none;
        }

        .breadcrumb__link--active {
          color: var(--ifm-color-text);
          font-weight: var(--ifm-font-weight-semibold);
          background-color: var(--ifm-color-primary-lightest);
          padding: var(--ifm-spacing-1) var(--ifm-spacing-2);
          border-radius: var(--ifm-spacing-1);
        }

        .breadcrumb__separator {
          margin: 0 var(--ifm-spacing-2);
          color: var(--ifm-color-text-light);
          display: flex;
          align-items: center;
        }

        .breadcrumb__separator-icon {
          width: 1rem;
          height: 1rem;
          color: var(--ifm-color-text-light);
        }
      `}</style>
    </nav>
  );
}