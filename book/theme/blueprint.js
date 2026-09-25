// Trim mdBook's theme picker to System / Light / Dark.
// "navy" is the class the Blueprint dark scheme is bound to in blueprint.css;
// book.js keeps handling clicks by button id, so removing entries is safe.
(function () {
    var list = document.getElementById('mdbook-theme-list');
    if (!list) { return; }
    var keep = {
        'mdbook-theme-default_theme': 'System',
        'mdbook-theme-light': 'Light',
        'mdbook-theme-navy': 'Dark'
    };
    list.querySelectorAll('button.theme').forEach(function (button) {
        if (keep[button.id]) {
            button.textContent = keep[button.id];
        } else {
            button.parentNode.remove();
        }
    });
})();
