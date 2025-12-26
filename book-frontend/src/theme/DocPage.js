import React from 'react';
import OriginalDocPage from '@theme-original/DocPage';
import { ThemeClassNames } from '@docusaurus/theme-common';
import clsx from 'clsx';
import { TranslationProvider } from '../contexts/TranslationContext';

export default function DocPage(props) {
  return (
    <TranslationProvider>
      <div className={clsx(ThemeClassNames.common.mainWrapper, 'doc-page-wrapper')}>
        <OriginalDocPage {...props} />
        <style jsx>{`
        .doc-page-wrapper {
          padding: var(--ifm-spacing-4) 0;
        }

        /* Enhanced styling for documentation content */
        .markdown {
          font-size: var(--ifm-font-size-base);
          line-height: var(--ifm-line-height-relaxed);
          color: var(--ifm-color-text);
        }

        .markdown h1 {
          font-size: var(--ifm-h1-font-size);
          font-weight: var(--ifm-font-weight-bold);
          line-height: var(--ifm-line-height-tight);
          margin-bottom: var(--ifm-spacing-6);
          color: var(--ifm-color-text);
        }

        .markdown h2 {
          font-size: var(--ifm-h2-font-size);
          font-weight: var(--ifm-font-weight-bold);
          line-height: var(--ifm-line-height-tight);
          margin-bottom: var(--ifm-spacing-5);
          margin-top: var(--ifm-spacing-8);
          padding-bottom: var(--ifm-spacing-2);
          border-bottom: 2px solid var(--ifm-color-primary-lighter);
          color: var(--ifm-color-text);
        }

        .markdown h3 {
          font-size: var(--ifm-h3-font-size);
          font-weight: var(--ifm-font-weight-semibold);
          line-height: var(--ifm-line-height-snug);
          margin-bottom: var(--ifm-spacing-4);
          margin-top: var(--ifm-spacing-6);
          color: var(--ifm-color-text);
        }

        .markdown h4 {
          font-size: var(--ifm-h4-font-size);
          font-weight: var(--ifm-font-weight-semibold);
          line-height: var(--ifm-line-height-base);
          margin-bottom: var(--ifm-spacing-3);
          color: var(--ifm-color-text);
        }

        .markdown p {
          margin-bottom: var(--ifm-spacing-4);
          color: var(--ifm-color-text);
        }

        .markdown a {
          color: var(--ifm-color-primary);
          text-decoration: none;
          position: relative;
        }

        .markdown a:hover {
          color: var(--ifm-color-primary-dark);
          text-decoration: underline;
        }

        .markdown a::after {
          content: '';
          position: absolute;
          bottom: -2px;
          left: 0;
          width: 100%;
          height: 1px;
          background-color: var(--ifm-color-primary);
          transform: scaleX(0);
          transition: transform 0.2s ease;
        }

        .markdown a:hover::after {
          transform: scaleX(1);
        }

        .markdown ul,
        .markdown ol {
          margin-bottom: var(--ifm-spacing-4);
          padding-left: var(--ifm-spacing-6);
        }

        .markdown li {
          margin-bottom: var(--ifm-spacing-2);
          line-height: var(--ifm-line-height-relaxed);
        }

        .markdown blockquote {
          border-left: 4px solid var(--ifm-color-primary);
          background-color: var(--ifm-color-background-alt);
          padding: var(--ifm-spacing-4) var(--ifm-spacing-6);
          margin: var(--ifm-spacing-4) 0;
          border-radius: 0 var(--ifm-spacing-2) var(--ifm-spacing-2) 0;
          font-style: italic;
          color: var(--ifm-color-text-secondary);
        }

        .markdown code {
          background-color: var(--ifm-color-code-background);
          color: var(--ifm-color-code-text);
          padding: var(--ifm-spacing-1) var(--ifm-spacing-2);
          border-radius: var(--ifm-spacing-1);
          font-size: var(--ifm-code-font-size);
        }

        .markdown pre {
          background-color: var(--ifm-color-code-background-dark);
          color: var(--ifm-color-text-inverse);
          padding: var(--ifm-spacing-4);
          border-radius: var(--ifm-spacing-2);
          overflow-x: auto;
          margin: var(--ifm-spacing-4) 0;
        }

        .markdown pre code {
          background: none;
          color: inherit;
          padding: 0;
        }

        .markdown table {
          width: 100%;
          border-collapse: collapse;
          margin: var(--ifm-spacing-4) 0;
          border-radius: var(--ifm-spacing-2);
          overflow: hidden;
          box-shadow: var(--ifm-shadow);
        }

        .markdown table th,
        .markdown table td {
          padding: var(--ifm-spacing-3) var(--ifm-spacing-4);
          text-align: left;
          border-bottom: 1px solid var(--ifm-color-border);
        }

        .markdown table th {
          background-color: var(--ifm-color-background-alt);
          font-weight: var(--ifm-font-weight-semibold);
        }

        .markdown table tr:last-child td {
          border-bottom: none;
        }

        .markdown hr {
          border: none;
          height: 1px;
          background-color: var(--ifm-color-border);
          margin: var(--ifm-spacing-6) 0;
        }

        /* Enhanced responsive design for documentation */
        @media (max-width: 768px) {
          .markdown h1 {
            font-size: var(--ifm-h2-font-size);
          }

          .markdown h2 {
            font-size: var(--ifm-h3-font-size);
          }

          .markdown h3 {
            font-size: var(--ifm-h4-font-size);
          }

          .markdown {
            padding: 0 var(--ifm-spacing-2);
          }
        }
      `}</style>
    </div>
    </TranslationProvider>
  );
}