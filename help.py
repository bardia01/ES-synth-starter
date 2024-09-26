import numpy as np

# write a function to generate a 1D gaussian kernel

def gaussian_kernel_1d(sigma, truncate=4.0):
    """Return a 1D Gaussian kernel.
    Parameters
    ----------
    sigma : float
        Standard deviation of the Gaussian kernel.
    truncate : float
        Truncate the kernel at this many standard deviations.
    Returns
    -------
    kernel : ndarray
        1D Gaussian kernel.
    """
    # calculate the kernel size
    size = int(truncate * sigma + 0.5)
    # create a 1D array of x values
    x = np.arange(-size, size + 1)
    # calculate the kernel
    kernel = np.exp(-0.5 * (x / sigma)**2) / (sigma * np.sqrt(2 * np.pi))
    # normalize the kernel
    kernel = kernel / np.sum(kernel)
    return kernel

print(gaussian_kernel_1d(1.0))