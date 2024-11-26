// app/contact/page.tsx

"use client"; // Ensure this file is client-side

import { Box, Button, Input, Textarea, VStack, Text } from "@chakra-ui/react";
import { useState } from "react";

const ContactUsPage = () => {
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [message, setMessage] = useState("");
  const [formError, setFormError] = useState("");
  const [formSuccess, setFormSuccess] = useState(false);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    // Reset feedback states
    setFormError("");
    setFormSuccess(false);

    // Basic validation
    if (!name || !email || !message) {
      setFormError("Please fill in all fields.");
      return;
    }

    // Simulate sending the message (replace with your actual API call)
    console.log({ name, email, message });

    // Set success state after "sending"
    setFormSuccess(true);

    // Reset the form after submission
    setName("");
    setEmail("");
    setMessage("");
  };

  return (
    <Box maxW="lg" mx="auto" py={8} px={4}>
      <VStack spacing={4} as="form" onSubmit={handleSubmit}>
        {/* Name input */}
        <Box id="name" width="full">
          <Input
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="Enter your name"
            isRequired
          />
        </Box>

        {/* Email input */}
        <Box id="email" width="full">
          <Input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="Enter your email"
            isRequired
          />
        </Box>

        {/* Message input */}
        <Box id="message" width="full">
          <Textarea
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            placeholder="Enter your message"
            rows={6}
            isRequired
          />
        </Box>

        {/* Display error or success message */}
        {formError && (
          <Text color="red.500" fontSize="sm">
            {formError}
          </Text>
        )}
        {formSuccess && (
          <Text color="green.500" fontSize="sm">
            Your message has been sent successfully!
          </Text>
        )}

        {/* Submit button */}
        <Button
          colorScheme="teal"
          type="submit"
          width="full"
        >
          Send Message
        </Button>
      </VStack>
    </Box>
  );
};

export default ContactUsPage;
