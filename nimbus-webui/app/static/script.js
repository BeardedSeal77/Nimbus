document.addEventListener('DOMContentLoaded', function() {
    const generateButton = document.getElementById('generate-button');
    const randomNumberDisplay = document.getElementById('random-number-display');
    const errorMessage = document.getElementById('error-message');

    generateButton.addEventListener('click', fetchRandomNumber);

    function fetchRandomNumber() {
        // Display loading state
        randomNumberDisplay.textContent = 'Loading...';
        randomNumberDisplay.classList.add('loading');
        errorMessage.style.display = 'none';
        
        // Disable button during request
        generateButton.disabled = true;
        
        // Make request to our Flask backend
        fetch('/get-random-number')
            .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error! Status: ${response.status}`);
                }
                return response.json();
            })
            .then(data => {
                // Display the random number
                if (data.number !== undefined) {
                    randomNumberDisplay.textContent = data.number;
                } else if (data.error) {
                    throw new Error(data.error);
                } else {
                    throw new Error('Unexpected response format');
                }
            })
            .catch(error => {
                // Display error message
                console.error('Error fetching random number:', error);
                errorMessage.textContent = `Error: ${error.message}`;
                errorMessage.style.display = 'block';
                randomNumberDisplay.textContent = 'Failed to get number';
            })
            .finally(() => {
                // Reset UI state
                randomNumberDisplay.classList.remove('loading');
                generateButton.disabled = false;
            });
    }
});
