import { initializeApp } from 'firebase/app';
import { getDatabase } from 'firebase/database';

// Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyDUYczWG0gDm2MqNhb0oh6iBRphYUcun9M",
  authDomain: "aavishkar-12bae.firebaseapp.com",
  databaseURL: "https://aavishkar-12bae-default-rtdb.firebaseio.com",
  projectId: "aavishkar-12bae",
  storageBucket: "aavishkar-12bae.firebasestorage.app",
  messagingSenderId: "408856217628",
  appId: "1:408856217628:web:8608c103afaf82b370b7d9",
  measurementId: "G-XBN56KW9E2"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);

// Get a reference to the database service
export const database = getDatabase(app);
