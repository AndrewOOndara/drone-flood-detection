"use client"

import { Box, Flex, Heading, Text, Button, Image, Stack } from "@chakra-ui/react";
import { useState } from "react";

const ProblemSection = () => {
  const [currentImage, setCurrentImage] = useState(0);

  const images = [
    "/flood_image1.jpg",
    "/flood_image2.jpeg",
    "/flood_image3.jpeg",
  ];

  const stats = {
    statistic1: "Over 100,000 vehicles are damaged in floods each year.",
    statistic2: "Flooded areas cause $17 billion in damage annually.",
    statistic3: "40% of flood-related deaths are due to driving through flooded roads.",
  };

  return (
    <Box py={16} px={8} bg="white">
      <Flex direction={{ base: "column", md: "row" }} align="center" justify="space-between">
        {/* Left Section: Images */}
        <Box flex="1" pr={{ base: 0, md: 8 }} mb={{ base: 8, md: 0 }}>
          <Image
            src={images[currentImage]}
            alt="Flooded Area"
            borderRadius="md"
            boxSize="full"
            objectFit="cover"
          />
        </Box>

        {/* Right Section: Text */}
        <Box flex="1" pr={50}>
          <Stack spacing={4} maxW="lg">
            <Heading as="h2" fontSize={36}  lineHeight={1} color="teal.500" fontWeight="bold">
              The Problem with Driving in Flooded Areas
            </Heading>
            <Text fontSize="lg" color="gray.700">
              Flooded roads present major risks for drivers, leading to significant damage and loss of life.
              <br />
              <br />
              <strong>Statistics:</strong>
              <ul>
                <li>{stats.statistic1}</li>
                <li>{stats.statistic2}</li>
                <li>{stats.statistic3}</li>
              </ul>
            </Text>
            <Button colorScheme="teal" size="lg" bg="#5a7adc" onClick={() => console.log("Take Action!")}>
              Learn How to Stay Safe
            </Button>
          </Stack>
        </Box>
      </Flex>
    </Box>
  );
};

export default ProblemSection;
