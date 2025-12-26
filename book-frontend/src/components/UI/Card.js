import React from 'react';
import clsx from 'clsx';

export default function Card({
  title,
  children,
  className,
  href,
  image,
  footer,
  variant = 'default',
  ...props
}) {
  const cardClasses = clsx(
    'card',
    `card--${variant}`,
    className
  );

  const cardContent = (
    <div className="card__inner">
      {image && (
        <div className="card__image">
          <img src={image} alt={title} />
        </div>
      )}
      {title && (
        <h3 className="card__header">
          {title}
        </h3>
      )}
      <div className="card__body">
        {children}
      </div>
      {footer && (
        <div className="card__footer">
          {footer}
        </div>
      )}
    </div>
  );

  return (
    <div className={cardClasses} {...props}>
      {href ? (
        <a href={href} className="card__link">
          {cardContent}
        </a>
      ) : (
        cardContent
      )}
      <style jsx>{`
        .card {
          border: 1px solid var(--ifm-color-border);
          border-radius: var(--ifm-spacing-2);
          overflow: hidden;
          transition: var(--ifm-transition-all);
          background-color: var(--ifm-color-background);
          box-shadow: var(--ifm-shadow);
          display: flex;
          flex-direction: column;
          height: 100%;
        }

        .card:hover {
          transform: translateY(-4px);
          box-shadow: var(--ifm-shadow-lg);
          border-color: var(--ifm-color-primary-lighter);
        }

        .card__inner {
          padding: var(--ifm-spacing-6);
          display: flex;
          flex-direction: column;
          height: 100%;
        }

        .card__image {
          margin-bottom: var(--ifm-spacing-4);
        }

        .card__image img {
          width: 100%;
          height: auto;
          border-radius: var(--ifm-spacing-1);
          max-height: 200px;
          object-fit: cover;
        }

        .card__header {
          font-size: var(--ifm-h4-font-size);
          font-weight: var(--ifm-font-weight-semibold);
          margin-bottom: var(--ifm-spacing-3);
          color: var(--ifm-color-text);
          line-height: var(--ifm-line-height-tight);
        }

        .card__body {
          flex: 1;
          margin-bottom: var(--ifm-spacing-4);
        }

        .card__footer {
          margin-top: auto;
          padding-top: var(--ifm-spacing-4);
          border-top: 1px solid var(--ifm-color-border);
        }

        .card__link {
          text-decoration: none;
          color: inherit;
          display: block;
          height: 100%;
        }

        .card__link:hover {
          text-decoration: none;
        }

        .card--elevated {
          box-shadow: var(--ifm-shadow-md);
        }

        .card--outlined {
          box-shadow: none;
          border: 2px solid var(--ifm-color-primary);
        }

        .card--primary {
          border-color: var(--ifm-color-primary);
          background-color: var(--ifm-color-primary-lightest);
        }

        .card--secondary {
          border-color: var(--ifm-color-secondary);
          background-color: var(--ifm-color-secondary-lightest);
        }
      `}</style>
    </div>
  );
}