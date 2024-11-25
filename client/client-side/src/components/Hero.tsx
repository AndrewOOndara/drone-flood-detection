// components/Hero.tsx
"use client"

import { Box, Flex, Heading, Text, Button, Image, Stack } from "@chakra-ui/react";
import { useRouter } from 'next/navigation';

const Hero = () => {
    const router = useRouter()
  return (
    <Box bg="white" px={8} py={16}>
      <Flex align="center" justify="space-between" direction={{ base: "column", md: "row" }}>
        {/* Left Section: Title and Subheader */}
        <Stack spacing={6} maxW="lg" textAlign={{ base: "center", md: "left" }}>
          <Heading as="h1" fontSize="64px" color="teal.500"  fontWeight="700" lineHeight="76.8px">
            Huge Puddle Industry
          </Heading>
          <Text fontSize="lg" color="gray.700">
            Saving you, one big puddle at a time.
          </Text>
          <Button colorScheme="teal" size="lg" bg="#5a7adc" width="168px" onClick={() => router.push('/path-planner')}>
            Try Now!
          </Button>
        </Stack>

        {/* Right Section: Image */}
        <Box mt={{ base: 10, md: 0 }} ml={{ md: 10 }} w={{ base: "80%", md: "50%" }}>
          <Image
            src="/hero_pic.png" // Replace with your image path
            alt="Huge Puddle Industry"
            borderRadius="md"
          />
        </Box>
      </Flex>
    </Box>
  );
};

export default Hero;
