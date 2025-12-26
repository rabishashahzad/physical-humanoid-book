import React from 'react';
import clsx from 'clsx';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useLocation } from '@docusaurus/router';

import DocItemPaginator from '@theme/DocItem/Paginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import DocItemFooter from '@theme/DocItem/Footer';
import DocItemContent from '@theme/DocItem/Content';

import styles from './styles.module.css';

export default function DocItemLayout({ children }) {
  const doc = useDoc();
  const { frontMatter, toc, metadata } = doc;
  const { wrapperClassName } = frontMatter;
  const { pathname } = useLocation();

  const isActiveTocLink = (id) => {
    if (typeof window === 'undefined') return false;
    return (
      pathname === metadata.permalink &&
      window.location.hash === `#${id}`
    );
  };

  return (
    <div className={clsx('docContainer', wrapperClassName)}>
      <div className="row">
        <div className={clsx('col', styles.docItemCol)}>
          <div className={styles.docItemContainer}>
            <article>
              <header>
                <div className={styles.docHeader}>
                  <DocVersionBanner />
                  <DocVersionBadge />
                  <h1 className={clsx(styles.docTitle, 'hero__title')}>
                    {metadata.title}
                  </h1>
                </div>
              </header>

              <div className={styles.docItemContent}>
                <DocItemContent>{children}</DocItemContent>
              </div>

              <footer className={styles.docFooter}>
                <DocItemFooter />
                <DocItemPaginator />
              </footer>
            </article>
          </div>
        </div>

        {toc && toc.length > 0 && (
          <div className="col col--3">
            <div className={clsx(styles.tableOfContents, 'thin-scrollbar')}>
              <div className={styles.tableOfContentsContent}>
                <h3 className={styles.tocTitle}>On this page</h3>
                <nav
                  aria-label="Table of Contents"
                  className={styles.tocNav}
                >
                  {toc.map((item) => (
                    <div key={item.id} className={styles.tocItem}>
                      <a
                        href={`#${item.id}`}
                        className={clsx(styles.tocLink, {
                          [styles.tocLinkActive]: isActiveTocLink(item.id),
                        })}
                        onClick={(e) => {
                          e.preventDefault();
                          const element = document.getElementById(item.id);
                          if (element) {
                            element.scrollIntoView({ behavior: 'smooth' });
                            window.history.pushState(null, '', `#${item.id}`);
                          }
                        }}
                      >
                        {item.value}
                      </a>
                    </div>
                  ))}
                </nav>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
