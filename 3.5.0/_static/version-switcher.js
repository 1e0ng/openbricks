/* Version switcher + archive banner for docs.openbricks.dev.
 *
 * The sidebar version pill (rendered by _templates/layout.html with
 * the firmware version Sphinx built against) opens a flyout listing
 * every published docs version from /versions.json at the site root
 * (generated at deploy time by scripts/docs-versions.py from the
 * docs-archive branch). Archived builds live under /X.Y.Z/ and are
 * frozen at their release; the root is always the latest.
 *
 * On an archived page (URL path starts with /X.Y.Z/) this also
 * injects a banner linking to the latest docs and a robots noindex
 * meta tag, so search engines index only the current version.
 *
 * The manifest URL is absolute: an archived page can't know the site
 * root relatively, and localhost preview builds simply have no
 * switcher (the pill stays as a static version label — no fake menu,
 * no fallback content).
 */
(function () {
  "use strict";

  var pill = document.getElementById("obk-version");
  if (!pill) {
    return;
  }
  var built = pill.getAttribute("data-version");

  var archiveMatch = window.location.pathname.match(
    /^\/(\d+\.\d+\.\d+)(\/|$)/);
  var archivedVersion = archiveMatch ? archiveMatch[1] : null;

  if (archivedVersion) {
    // Keep crawlers on the latest docs only.
    var meta = document.createElement("meta");
    meta.name = "robots";
    meta.content = "noindex";
    document.head.appendChild(meta);

    var banner = document.createElement("div");
    banner.className = "obk-version-banner";
    banner.innerHTML =
      "You are reading the documentation archive for firmware v" +
      archivedVersion +
      '. <a href="/">Go to the latest documentation.</a>';
    var content = document.querySelector(".wy-nav-content") ||
      document.body;
    content.insertBefore(banner, content.firstChild);
  }

  fetch("https://docs.openbricks.dev/versions.json")
    .then(function (r) {
      if (!r.ok) {
        throw new Error("versions.json HTTP " + r.status);
      }
      return r.json();
    })
    .then(function (manifest) {
      var menu = document.createElement("div");
      menu.className = "obk-version-menu";
      menu.setAttribute("hidden", "hidden");

      function addRow(label, url, pdf, isCurrent) {
        var row = document.createElement("div");
        row.className = "obk-version-row" +
          (isCurrent ? " obk-version-row-current" : "");
        var link = document.createElement("a");
        link.href = url;
        link.textContent = label;
        row.appendChild(link);
        if (pdf) {
          var pdfLink = document.createElement("a");
          pdfLink.className = "obk-version-pdf";
          pdfLink.href = pdf;
          pdfLink.textContent = "PDF";
          row.appendChild(pdfLink);
        }
        menu.appendChild(row);
      }

      addRow("latest (v" + manifest.latest + ")", "/",
        "/openbricks-docs.pdf", archivedVersion === null);
      manifest.versions.forEach(function (v) {
        addRow("v" + v.version, v.url, v.pdf,
          v.version === archivedVersion);
      });

      pill.parentNode.insertBefore(menu, pill.nextSibling);
      pill.classList.add("obk-version-has-menu");
      pill.addEventListener("click", function (ev) {
        ev.preventDefault();
        if (menu.hasAttribute("hidden")) {
          menu.removeAttribute("hidden");
        } else {
          menu.setAttribute("hidden", "hidden");
        }
      });
      document.addEventListener("click", function (ev) {
        if (!pill.contains(ev.target) && !menu.contains(ev.target)) {
          menu.setAttribute("hidden", "hidden");
        }
      });
    })
    .catch(function () {
      // No manifest (localhost preview, offline bundle): the pill is
      // a plain version label, nothing more.
      pill.classList.remove("obk-version-has-menu");
    });
})();
