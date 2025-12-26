import React from 'react';
import clsx from 'clsx';

export default function Alert({
  children,
  variant = 'info',
  title,
  icon,
  closable = false,
  className,
  onClose,
  ...props
}) {
  const [isVisible, setIsVisible] = React.useState(true);

  const handleClose = () => {
    if (onClose) {
      onClose();
    } else {
      setIsVisible(false);
    }
  };

  if (!isVisible) {
    return null;
  }

  const alertClasses = clsx(
    'alert',
    `alert--${variant}`,
    {
      'alert--closable': closable,
    },
    className
  );

  return (
    <div className={alertClasses} role="alert" {...props}>
      <div className="alert__content">
        {icon && <div className="alert__icon">{icon}</div>}
        <div className="alert__body">
          {title && <h3 className="alert__title">{title}</h3>}
          <div className="alert__message">{children}</div>
        </div>
        {closable && (
          <button
            className="alert__close-button"
            onClick={handleClose}
            aria-label="Close alert"
          >
            <svg className="alert__close-icon" width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        )}
      </div>
      <style jsx>{`
        .alert {
          border: 1px solid var(--ifm-color-border);
          border-radius: var(--ifm-spacing-2);
          padding: var(--ifm-spacing-4);
          margin-bottom: var(--ifm-spacing-4);
          display: flex;
          align-items: flex-start;
          gap: var(--ifm-spacing-3);
        }

        .alert__content {
          display: flex;
          align-items: flex-start;
          gap: var(--ifm-spacing-3);
          flex: 1;
        }

        .alert__icon {
          margin-top: var(--ifm-spacing-1);
        }

        .alert__body {
          flex: 1;
        }

        .alert__title {
          font-size: var(--ifm-font-size-lg);
          font-weight: var(--ifm-font-weight-semibold);
          margin: 0 0 var(--ifm-spacing-2) 0;
          color: var(--ifm-color-text);
        }

        .alert__message {
          margin: 0;
          color: var(--ifm-color-text);
          line-height: var(--ifm-line-height-relaxed);
        }

        .alert__close-button {
          background: none;
          border: none;
          cursor: pointer;
          padding: var(--ifm-spacing-1);
          border-radius: var(--ifm-spacing-1);
          color: var(--ifm-color-text);
          transition: var(--ifm-transition-colors);
        }

        .alert__close-button:hover {
          background-color: var(--ifm-color-background-alt);
        }

        .alert__close-icon {
          width: 1.25rem;
          height: 1.25rem;
        }

        /* Variants */
        .alert--info {
          background-color: var(--ifm-color-primary-lightest);
          border-color: var(--ifm-color-primary-lighter);
          color: var(--ifm-color-primary-darker);
        }

        .alert--success {
          background-color: var(--ifm-color-success-lightest);
          border-color: var(--ifm-color-success-lighter);
          color: var(--ifm-color-success-darker);
        }

        .alert--warning {
          background-color: var(--ifm-color-warning-lightest);
          border-color: var(--ifm-color-warning-lighter);
          color: var(--ifm-color-warning-darker);
        }

        .alert--danger {
          background-color: var(--ifm-color-danger-lightest);
          border-color: var(--ifm-color-danger-lighter);
          color: var(--ifm-color-danger-darker);
        }
      `}</style>
    </div>
  );
}