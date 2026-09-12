// Firebase Analytics (page views for docs.openbricks.dev).
import { initializeApp } from "https://www.gstatic.com/firebasejs/12.17.1/firebase-app.js";
import { getAnalytics } from "https://www.gstatic.com/firebasejs/12.17.1/firebase-analytics.js";

const firebaseConfig = {
  apiKey: "AIzaSyCxBJZrMJ2v4IKcIWqvNFEZa_FzRWCuSbs",
  authDomain: "openbricks-dev.firebaseapp.com",
  projectId: "openbricks-dev",
  storageBucket: "openbricks-dev.firebasestorage.app",
  messagingSenderId: "935642854203",
  appId: "1:935642854203:web:e7882c5cc259ace2d47d1e",
  measurementId: "G-8P0NNXZHER"
};

const app = initializeApp(firebaseConfig);
getAnalytics(app);
